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

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
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

class ScopedSpinner
{
public:
  explicit ScopedSpinner(rclcpp::Executor & exe)
  : thread_([this, &exe]() {while (!finish_) {exe.spin_once(10ms);}}) {}
  ~ScopedSpinner()
  {
    finish_ = true;
    thread_.join();
  }

private:
  std::atomic<bool> finish_ {false};
  std::thread thread_;
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
    spinner_ = std::make_unique<ScopedSpinner>(exe_);
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
  rclcpp::executors::SingleThreadedExecutor exe_;
  std::unique_ptr<ScopedSpinner> spinner_;
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
  spinner_ = std::make_unique<ScopedSpinner>(exe_);

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
  SilentExecutor()
  {
    node = rclcpp::Node::make_shared("silent_executor");
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
  rclcpp_action::Server<ExecutePlan>::SharedPtr server;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>>> goals;
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
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(silent->node);
  auto spinner = std::make_unique<ScopedSpinner>(exe);

  auto client = std::make_shared<plansys2::ExecutorClient>();
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
  spinner = std::make_unique<ScopedSpinner>(exe);
  ASSERT_TRUE(client->start_plan_execution(one_move()));
  ASSERT_TRUE(client->execute_and_check_plan());
  spinner.reset();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
