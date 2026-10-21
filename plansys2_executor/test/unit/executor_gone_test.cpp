// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Performers and executor clients notice when the executor goes down (#446). The
// executor disappears without cancelling anything, as when its process dies.
// Each scenario runs in its own process (see CMakeLists.txt).

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT
using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace
{

// Finishes each action after `ticks` ticks at 20 Hz
class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction(const std::string & name, int ticks)
  : ActionExecutorClient(name), ticks_needed_(ticks)
  {
    set_parameter({"action_name", "move"});
    set_parameter({"rate", 20.0});
  }

  std::atomic<int> started {0};
  std::atomic<int> finished {0};

private:
  void do_work() override
  {
    if (ticks_ == 0) {
      started++;
    }
    if (++ticks_ >= ticks_needed_) {
      ticks_ = 0;
      finished++;
      finish(true, 1.0, "done");
    } else {
      send_feedback(static_cast<float>(ticks_) / ticks_needed_, "moving");
    }
  }

  int ticks_needed_;
  int ticks_ {0};
};

bool wait_until(const std::function<bool()> & condition, std::chrono::seconds timeout)
{
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
  return true;
}

}  // namespace

class PerformerSupervisionTest : public ::testing::Test
{
protected:
  // A performer, and an "executor" node asking it for one move
  void start(int performer_ticks)
  {
    performer_ = std::make_shared<MoveAction>("move_performer", performer_ticks);
    executor_node_ = rclcpp_lifecycle::LifecycleNode::make_shared("fake_executor");
    exe_.add_node(performer_->get_node_base_interface());
    exe_.add_node(executor_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
    performer_->trigger_transition(Transition::TRANSITION_CONFIGURE);

    action_ = plansys2::ActionExecutor::make_shared("(move r2d2 a b)", executor_node_);
    ASSERT_TRUE(
      wait_until(
        [this]() {
          action_->tick(executor_node_->now());
          return performer_->started > 0;
        }, 10s));
  }

  bool performer_active()
  {
    return performer_->get_current_state().id() == State::PRIMARY_STATE_ACTIVE;
  }

  void TearDown() override
  {
    spinner_.reset();
    action_.reset();
  }

  std::shared_ptr<MoveAction> performer_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr executor_node_;
  plansys2::ActionExecutor::Ptr action_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

TEST_F(PerformerSupervisionTest, performer_stops_when_its_executor_is_gone)
{
  start(200);  // 10 s per action
  ASSERT_TRUE(performer_active());

  // The executor vanishes without cancelling anything. Not destroyed while spinning: rclcpp
  // with Fast DDS may then read a freed reader of it in the wait set
  spinner_.reset();
  exe_.remove_node(executor_node_->get_node_base_interface());
  action_.reset();
  executor_node_.reset();
  spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);

  ASSERT_TRUE(wait_until([this]() {return !performer_active();}, 8s));
  ASSERT_EQ(performer_->finished, 0);

  // It is ready for the next executor that asks
  ASSERT_EQ(performer_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(PerformerSupervisionTest, performer_keeps_running_while_its_executor_lives)
{
  // Longer than the few seconds the check needs to give up on a missing executor
  start(120);  // 6 s
  ASSERT_TRUE(
    wait_until(
      [this]() {
        action_->tick(executor_node_->now());
        return performer_->finished > 0;
      }, 15s));
  ASSERT_EQ(action_->get_internal_status(), plansys2::ActionExecutor::SUCCESS);
}

// An execute_plan server that accepts goals and never ends them
class SilentExecutor
{
public:
  // With get_plan, it says whether it runs the plan of its last goal (runs)
  explicit SilentExecutor(bool with_get_plan = false)
  {
    node = rclcpp::Node::make_shared("silent_executor");
    state = plansys2::testing::active_state(*node, "executor");
    if (with_get_plan) {
      get_plan = node->create_service<plansys2_msgs::srv::GetPlan>(
        "executor/get_plan",
        [this](const plansys2_msgs::srv::GetPlan::Request::SharedPtr,
        plansys2_msgs::srv::GetPlan::Response::SharedPtr response) {
          response->success = runs && !goals.empty();
          if (response->success) {
            response->plan = goals.back()->get_goal()->plan;
          }
        });
    }
    server = rclcpp_action::create_server<ExecutePlan>(
      node, "execute_plan",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecutePlan::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>> goal) {
        goals.push_back(goal);
      });
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr state;
  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan;
  rclcpp_action::Server<ExecutePlan>::SharedPtr server;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>>> goals;
  std::atomic<bool> runs {true};
};

plansys2_msgs::msg::Plan one_move()
{
  plansys2_msgs::msg::Plan plan;
  plansys2_msgs::msg::PlanItem item;
  item.action = "(move r2d2 a b)";
  item.duration = 1.0;
  plan.items.push_back(item);
  return plan;
}

TEST(ExecutorClientSupervision, client_stops_waiting_when_the_executor_is_gone)
{
  auto silent = std::make_unique<SilentExecutor>();
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(silent->node);
  auto spinner = std::make_unique<plansys2::testing::ExecutorThread>(exe);

  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto client = std::make_shared<plansys2::ExecutorClient>(client_node);
  ASSERT_TRUE(client->start_plan_execution(one_move()));

  // While the executor lives, the plan is still running
  auto alive_until = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < alive_until) {
    ASSERT_TRUE(client->execute_and_check_plan());
    std::this_thread::sleep_for(100ms);
  }

  // Gone: the client stops waiting, with no result. Destroying a goal handle aborts
  // its goal, which a dead process never does: they outlive the server
  spinner.reset();
  static auto * orphan_goals = new decltype(silent->goals)(silent->goals);
  (void)orphan_goals;
  silent.reset();
  ASSERT_TRUE(wait_until([&]() {return !client->execute_and_check_plan();}, 10s));
  ASSERT_FALSE(client->getResult().has_value());

  // A new executor takes a new plan from the same client
  silent = std::make_unique<SilentExecutor>();
  exe.add_node(silent->node);
  spinner = std::make_unique<plansys2::testing::ExecutorThread>(exe);
  ASSERT_TRUE(client->start_plan_execution(one_move()));
  ASSERT_TRUE(client->execute_and_check_plan());
  spinner.reset();
}

// A restarted executor has an action server, but not our goal: no result would ever come
TEST(ExecutorClientSupervision, client_stops_waiting_when_the_executor_forgets_the_plan)
{
  SilentExecutor silent(true);
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(silent.node);
  plansys2::testing::ExecutorThread spinner(exe);

  auto client_node = rclcpp::Node::make_shared("forget_client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto client = std::make_shared<plansys2::ExecutorClient>(client_node);
  ASSERT_TRUE(client->start_plan_execution(one_move()));

  // While it runs our plan, the plan goes on
  auto running_until = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < running_until) {
    ASSERT_TRUE(client->execute_and_check_plan());
    std::this_thread::sleep_for(100ms);
  }

  // It forgets it (as a restarted one): the client stops waiting, with no result
  silent.runs = false;
  auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait_until([&]() {return !client->execute_and_check_plan();}, 10s));
  EXPECT_GE(std::chrono::steady_clock::now() - start, 2s);
  ASSERT_FALSE(client->getResult().has_value());

  // A new plan from the same client runs
  silent.runs = true;
  ASSERT_TRUE(client->start_plan_execution(one_move()));
  ASSERT_TRUE(client->execute_and_check_plan());
}

// An execute_plan server made of its parts, to lose the answers to get_result at will
class LossyExecutor
{
public:
  using Impl = ExecutePlan::Impl;

  LossyExecutor()
  {
    node = rclcpp::Node::make_shared("lossy_executor");
    state = plansys2::testing::active_state(*node, "executor");
    send_goal = node->create_service<Impl::SendGoalService>(
      "execute_plan/_action/send_goal",
      [this](const Impl::SendGoalService::Request::SharedPtr request,
      Impl::SendGoalService::Response::SharedPtr response) {
        plan = request->goal.plan;
        response->accepted = true;
      });
    cancel = node->create_service<action_msgs::srv::CancelGoal>(
      "execute_plan/_action/cancel_goal",
      [](const action_msgs::srv::CancelGoal::Request::SharedPtr,
      action_msgs::srv::CancelGoal::Response::SharedPtr) {});
    get_result = node->create_service<Impl::GetResultService>(
      "execute_plan/_action/get_result",
      [this](std::shared_ptr<rmw_request_id_t> header,
      const Impl::GetResultService::Request::SharedPtr) {
        results_asked++;
        if (status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
          return;  // Lost on the way, as Fast DDS does at times
        }
        Impl::GetResultService::Response response;
        response.status = status;
        response.result.result = ExecutePlan::Result::SUCCESS;
        get_result->send_response(*header, response);
      });
    feedback = node->create_publisher<Impl::FeedbackMessage>("execute_plan/_action/feedback", 10);
    status_pub = node->create_publisher<action_msgs::msg::GoalStatusArray>(
      "execute_plan/_action/status", rclcpp::QoS(1).transient_local());
    get_plan = node->create_service<plansys2_msgs::srv::GetPlan>(
      "executor/get_plan",
      [this](const plansys2_msgs::srv::GetPlan::Request::SharedPtr,
      plansys2_msgs::srv::GetPlan::Response::SharedPtr response) {
        response->success = status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
        response->plan = plan;
      });
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr state;
  rclcpp::Service<Impl::SendGoalService>::SharedPtr send_goal;
  rclcpp::Service<action_msgs::srv::CancelGoal>::SharedPtr cancel;
  rclcpp::Service<Impl::GetResultService>::SharedPtr get_result;
  rclcpp::Publisher<Impl::FeedbackMessage>::SharedPtr feedback;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub;
  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan;
  plansys2_msgs::msg::Plan plan;
  // EXECUTING: runs the plan and answers no get_result; else, ended (or unknown) with it
  std::atomic<int8_t> status {action_msgs::msg::GoalStatus::STATUS_EXECUTING};
  std::atomic<int> results_asked {0};
};

// The plan ends, but the answer with its result is lost: the client asks for it again
TEST(ExecutorClientSupervision, a_lost_result_is_asked_for_again)
{
  for (const int8_t end : {action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
      action_msgs::msg::GoalStatus::STATUS_ABORTED})
  {
    SCOPED_TRACE(static_cast<int>(end));
    LossyExecutor lossy;
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(lossy.node);
    plansys2::testing::ExecutorThread spinner(exe);

    auto client_node = rclcpp::Node::make_shared("lossy_client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto client = std::make_shared<plansys2::ExecutorClient>(client_node);
    ASSERT_TRUE(client->start_plan_execution(one_move()));

    // Running, its first get_result never answered
    auto running_until = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < running_until) {
      ASSERT_TRUE(client->execute_and_check_plan());
      std::this_thread::sleep_for(100ms);
    }
    ASSERT_GE(lossy.results_asked.load(), 1);

    // Ended: the result comes, as if it had not been lost
    lossy.status = end;
    ASSERT_TRUE(wait_until([&]() {return !client->execute_and_check_plan();}, 10s));
    ASSERT_TRUE(client->getResult().has_value());
    EXPECT_EQ(client->getResult()->result, ExecutePlan::Result::SUCCESS);
    EXPECT_GE(lossy.results_asked.load(), 2);
  }
}

// A restarted executor does not know our goal: the plan is lost, without waiting for more
TEST(ExecutorClientSupervision, a_goal_unknown_to_the_executor_is_lost)
{
  LossyExecutor lossy;
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(lossy.node);
  plansys2::testing::ExecutorThread spinner(exe);

  auto client_node = rclcpp::Node::make_shared("unknown_client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto client = std::make_shared<plansys2::ExecutorClient>(client_node);
  ASSERT_TRUE(client->start_plan_execution(one_move()));
  ASSERT_TRUE(client->execute_and_check_plan());

  lossy.status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait_until([&]() {return !client->execute_and_check_plan();}, 10s));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 7s);
  ASSERT_FALSE(client->getResult().has_value());

  // A new plan from the same client runs
  lossy.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  ASSERT_TRUE(client->start_plan_execution(one_move()));
  ASSERT_TRUE(client->execute_and_check_plan());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
