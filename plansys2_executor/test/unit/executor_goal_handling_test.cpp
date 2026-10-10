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

// Every execute_plan goal ends, with a consistent result, whatever arrives meanwhile (#436).
// Each scenario runs in its own process (see CMakeLists.txt).

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;  // NOLINT
using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecutePlan>;
using ResultCode = rclcpp_action::ResultCode;
using Transition = lifecycle_msgs::msg::Transition;
using ActionExecutionInfo = plansys2_msgs::msg::ActionExecutionInfo;

namespace
{

// Ends each action after `ticks` ticks, succeeding or failing as `succeed` says
class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction(const std::string & name, int ticks)
  : ActionExecutorClient(name), ticks_needed_(ticks) {}

  std::atomic<int> ticks_needed_;
  std::atomic<bool> succeed {true};
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
      finish(succeed, 1.0, succeed ? "done" : "failed");
    } else {
      send_feedback(static_cast<float>(ticks_) / ticks_needed_, "working");
    }
  }

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

const std::vector<std::string> kZones = {"wheels_zone", "steering_wheels_zone", "assembly_zone"};

plansys2_msgs::msg::Plan move_plan(const std::vector<std::string> & zones)
{
  plansys2_msgs::msg::Plan plan;
  for (size_t i = 0; i + 1 < zones.size(); i++) {
    plansys2_msgs::msg::PlanItem item;
    item.time = 5.001 * i;
    item.action = "(move r2d2 " + zones[i] + " " + zones[i + 1] + ")";
    item.duration = 5.0;
    plan.items.push_back(item);
  }
  return plan;
}

}  // namespace

class ExecutorGoalHandlingTest : public ::testing::Test
{
protected:
  // The performer is left inactive, waiting for requests; or not even configured
  void start(int performer_ticks, bool configure_performer = true)
  {
    std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_executor").string();
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    executor_node_ = std::make_shared<plansys2::ExecutorNode>();
    move_node_ = std::make_shared<MoveAction>("move_performer", performer_ticks);
    move_node_->set_parameter({"action_name", "move"});
    move_node_->set_parameter({"rate", 20.0});
    domain_node_->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});

    client_node_ = rclcpp::Node::make_shared("goal_handling_client");
    action_client_ = rclcpp_action::create_client<ExecutePlan>(client_node_, "execute_plan");

    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    for (auto node : lifecycle_nodes()) {
      exe_->add_node(node->get_node_base_interface());
    }
    exe_->add_node(client_node_);
    spinner_ = std::make_unique<ScopedSpinner>(*exe_);

    for (rclcpp_lifecycle::LifecycleNode::SharedPtr node : {
      std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(domain_node_),
      std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(problem_node_),
      std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(executor_node_)})
    {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
      node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }
    if (configure_performer) {
      move_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    }

    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
    ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("r2d2", "robot")));
    for (const auto & zone : kZones) {
      ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance(zone, "zone")));
    }
    ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate("(battery_full r2d2)")));
    place_robot("wheels_zone");
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
  }

  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> lifecycle_nodes()
  {
    return {domain_node_, problem_node_, executor_node_, move_node_};
  }

  void TearDown() override
  {
    problem_client_.reset();
    spinner_.reset();
    exe_.reset();
    action_client_.reset();
    client_node_.reset();
    executor_node_.reset();
    move_node_.reset();
    problem_node_.reset();
    domain_node_.reset();
  }

  // An interrupted move leaves the robot nowhere: put it back at `zone`, available
  void place_robot(const std::string & zone)
  {
    for (const auto & other : kZones) {
      problem_client_->removePredicate(plansys2::Predicate("(robot_at r2d2 " + other + ")"));
    }
    ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate("(robot_at r2d2 " + zone + ")")));
    ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate("(robot_available r2d2)")));
  }

  bool robot_at(const std::string & zone)
  {
    return problem_client_->existPredicate(plansys2::Predicate("(robot_at r2d2 " + zone + ")"));
  }

  std::shared_future<GoalHandle::SharedPtr> send_async(const std::vector<std::string> & zones)
  {
    ExecutePlan::Goal goal;
    goal.plan = move_plan(zones);
    return action_client_->async_send_goal(goal);
  }

  GoalHandle::SharedPtr accepted(std::shared_future<GoalHandle::SharedPtr> future)
  {
    if (future.wait_for(5s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  GoalHandle::SharedPtr send(const std::vector<std::string> & zones)
  {
    return accepted(send_async(zones));
  }

  std::optional<GoalHandle::WrappedResult> result_of(
    const GoalHandle::SharedPtr & handle, std::chrono::seconds timeout = 30s)
  {
    auto future = action_client_->async_get_result(handle);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return {};
    }
    return future.get();
  }

  bool wait_until(const std::function<bool()> & condition, std::chrono::seconds timeout = 10s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  bool cancel(const GoalHandle::SharedPtr & handle)
  {
    auto future = action_client_->async_cancel_goal(handle);
    return future.wait_for(5s) == std::future_status::ready;
  }

  // The executor still works: a plan from `from` to `to` succeeds
  void expect_plan_succeeds(const std::string & from, const std::string & to)
  {
    place_robot(from);
    auto handle = send({from, to});
    ASSERT_NE(handle, nullptr);
    auto result = result_of(handle);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->code, ResultCode::SUCCEEDED);
    ASSERT_EQ(result->result->result, ExecutePlan::Result::SUCCESS);
    ASSERT_TRUE(robot_at(to));
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  std::shared_ptr<plansys2::ExecutorNode> executor_node_;
  std::shared_ptr<MoveAction> move_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<ExecutePlan>::SharedPtr action_client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::unique_ptr<ScopedSpinner> spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
};

TEST_F(ExecutorGoalHandlingTest, succeeded_plan_reports_only_its_actions)
{
  start(3);
  auto handle = send({"wheels_zone", "steering_wheels_zone", "assembly_zone"});
  ASSERT_NE(handle, nullptr);
  auto result = result_of(handle);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->code, ResultCode::SUCCEEDED);
  ASSERT_EQ(result->result->result, ExecutePlan::Result::SUCCESS);

  // No INIT pseudo-action (":0") among them
  ASSERT_EQ(result->result->action_execution_status.size(), 2u);
  for (const auto & status : result->result->action_execution_status) {
    EXPECT_NE(status.action_full_name, ":0");
    EXPECT_EQ(status.status, ActionExecutionInfo::SUCCEEDED);
  }
  ASSERT_TRUE(robot_at("assembly_zone"));
}

TEST_F(ExecutorGoalHandlingTest, failed_plan_is_aborted_with_failure)
{
  start(3);
  move_node_->succeed = false;
  auto handle = send({"wheels_zone", "assembly_zone"});
  ASSERT_NE(handle, nullptr);
  auto result = result_of(handle);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->code, ResultCode::ABORTED);
  ASSERT_EQ(result->result->result, ExecutePlan::Result::FAILURE);
  ASSERT_EQ(result->result->action_execution_status.size(), 1u);
  ASSERT_EQ(result->result->action_execution_status[0].status, ActionExecutionInfo::FAILED);

  move_node_->succeed = true;
  expect_plan_succeeds("wheels_zone", "assembly_zone");
}

TEST_F(ExecutorGoalHandlingTest, cancelled_plan_stops_its_actions)
{
  start(60);  // 3 s per action
  auto handle = send({"wheels_zone", "assembly_zone"});
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_until([this]() {return move_node_->started > 0;}));
  ASSERT_TRUE(cancel(handle));

  auto result = result_of(handle, 10s);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->code, ResultCode::CANCELED);
  ASSERT_EQ(result->result->result, ExecutePlan::Result::FAILURE);

  // The action never completed, even after its 3 s
  std::this_thread::sleep_for(3s);
  ASSERT_EQ(move_node_->finished, 0);

  move_node_->ticks_needed_ = 3;
  expect_plan_succeeds("wheels_zone", "assembly_zone");
}

TEST_F(ExecutorGoalHandlingTest, cancelled_plan_does_not_start_actions_later)
{
  // No performer answers yet: the action is still looking for one when cancelled
  start(3, false);
  auto handle = send({"wheels_zone", "assembly_zone"});
  ASSERT_NE(handle, nullptr);
  std::this_thread::sleep_for(1500ms);
  ASSERT_TRUE(cancel(handle));
  auto result = result_of(handle, 10s);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->code, ResultCode::CANCELED);

  // A performer appearing afterwards gets nothing to execute
  move_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  std::this_thread::sleep_for(3s);
  ASSERT_EQ(move_node_->started, 0);

  expect_plan_succeeds("wheels_zone", "assembly_zone");
}

TEST_F(ExecutorGoalHandlingTest, new_goal_preempts_the_running_one)
{
  start(60);
  auto first = send({"wheels_zone", "assembly_zone"});
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(wait_until([this]() {return move_node_->started > 0;}));

  move_node_->ticks_needed_ = 3;
  place_robot("wheels_zone");
  auto second = send({"wheels_zone", "steering_wheels_zone"});
  ASSERT_NE(second, nullptr);

  auto first_result = result_of(first, 10s);
  ASSERT_TRUE(first_result.has_value());
  ASSERT_EQ(first_result->code, ResultCode::ABORTED);
  ASSERT_EQ(first_result->result->result, ExecutePlan::Result::PREEMPT);

  auto second_result = result_of(second);
  ASSERT_TRUE(second_result.has_value());
  ASSERT_EQ(second_result->code, ResultCode::SUCCEEDED);
  ASSERT_EQ(second_result->result->result, ExecutePlan::Result::SUCCESS);
  ASSERT_TRUE(robot_at("steering_wheels_zone"));
}

// Goals sent back to back, while idle and while a plan runs: only the last one executes,
// and every replaced goal gets its result instead of leaving its client waiting
TEST_F(ExecutorGoalHandlingTest, goals_in_a_row_all_end)
{
  start(60);
  for (bool while_running : {false, true}) {
    if (while_running) {
      place_robot("wheels_zone");
      ASSERT_NE(send({"wheels_zone", "assembly_zone"}), nullptr);
      std::this_thread::sleep_for(500ms);
    }

    place_robot("wheels_zone");
    std::vector<std::shared_future<GoalHandle::SharedPtr>> futures;
    for (int i = 0; i < 3; i++) {
      futures.push_back(send_async({"wheels_zone", "steering_wheels_zone"}));
    }
    std::vector<GoalHandle::SharedPtr> handles;
    for (auto & future : futures) {
      handles.push_back(accepted(future));
      ASSERT_NE(handles.back(), nullptr);
    }

    for (size_t i = 0; i + 1 < handles.size(); i++) {
      auto result = result_of(handles[i], 10s);
      ASSERT_TRUE(result.has_value()) << "goal " << i << " never ended";
      ASSERT_EQ(result->code, ResultCode::ABORTED);
      ASSERT_EQ(result->result->result, ExecutePlan::Result::PREEMPT);
    }

    // The last one runs (or fails, if an earlier one already started its move): stop it
    cancel(handles.back());
    auto last = result_of(handles.back(), 10s);
    ASSERT_TRUE(last.has_value());
    ASSERT_TRUE(
      last->code == ResultCode::CANCELED ||
      (last->code == ResultCode::ABORTED &&
      last->result->result == ExecutePlan::Result::FAILURE));
  }

  move_node_->ticks_needed_ = 3;
  expect_plan_succeeds("wheels_zone", "assembly_zone");
}

TEST_F(ExecutorGoalHandlingTest, cancel_a_goal_waiting_to_start)
{
  start(60);
  auto running = send({"wheels_zone", "assembly_zone"});
  ASSERT_NE(running, nullptr);
  ASSERT_TRUE(wait_until([this]() {return move_node_->started > 0;}));

  // Cancelled right after being accepted, before or after the executor takes it
  place_robot("wheels_zone");
  auto waiting = send({"wheels_zone", "steering_wheels_zone"});
  ASSERT_NE(waiting, nullptr);
  ASSERT_TRUE(cancel(waiting));

  auto waiting_result = result_of(waiting, 10s);
  ASSERT_TRUE(waiting_result.has_value());
  ASSERT_EQ(waiting_result->code, ResultCode::CANCELED);

  auto running_result = result_of(running, 10s);
  ASSERT_TRUE(running_result.has_value());
  ASSERT_EQ(running_result->code, ResultCode::ABORTED);
  ASSERT_EQ(running_result->result->result, ExecutePlan::Result::PREEMPT);

  move_node_->ticks_needed_ = 3;
  expect_plan_succeeds("wheels_zone", "assembly_zone");
}

// A cancel that arrives as the plan ends must not cancel the next goal: it used to be
// applied to it, and calling canceled() on a goal not canceling terminated the process
TEST_F(ExecutorGoalHandlingTest, cancel_as_the_plan_ends_does_not_reach_the_next_goal)
{
  start(2);
  std::string from = "wheels_zone";
  std::string to = "assembly_zone";
  for (int i = 0; i < 15; i++) {
    place_robot(from);
    int finished = move_node_->finished;
    auto handle = send({from, to});
    ASSERT_NE(handle, nullptr);

    // Cancel as soon as the performer finishes its action, when the plan is ending
    auto deadline = std::chrono::steady_clock::now() + 10s;
    while (move_node_->finished == finished && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::yield();
    }
    action_client_->async_cancel_goal(handle);

    auto result = result_of(handle, 10s);
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->code == ResultCode::SUCCEEDED || result->code == ResultCode::CANCELED);

    // The next goal is not affected
    expect_plan_succeeds(from, to);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
