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

// The executor thread follows the lifecycle and the node can be destroyed (#435, #430).
// Each scenario runs in its own process (see CMakeLists.txt).

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;  // NOLINT
using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace
{

// Finishes the action after `ticks` ticks; counts the actions it completed
class CountingAction : public plansys2::ActionExecutorClient
{
public:
  CountingAction(const std::string & name, int ticks)
  : ActionExecutorClient(name), ticks_needed_(ticks) {}

  std::atomic<int> finished {0};

private:
  void do_work() override
  {
    if (++ticks_ >= ticks_needed_) {
      ticks_ = 0;
      finished++;
      finish(true, 1.0, "done");
    } else {
      send_feedback(static_cast<float>(ticks_) / ticks_needed_, "working");
    }
  }

  int ticks_needed_;
  int ticks_ {0};
};

class ScopedSpinner
{
public:
  explicit ScopedSpinner(rclcpp::Executor & exe)
  : thread_([this, &exe]() {
        while (!finish_) {
          try {
            exe.spin_once(10ms);
          } catch (const std::exception & e) {
            // As executor_node's main: the action server may throw while rclcpp shuts down
            if (rclcpp::ok()) {
              ADD_FAILURE() << "Spinning failed: " << e.what();
            }
            return;
          }
        }
      }) {}
  ~ScopedSpinner()
  {
    finish_ = true;
    thread_.join();
  }

private:
  std::atomic<bool> finish_ {false};
  std::thread thread_;
};

plansys2_msgs::msg::Plan move_plan(const std::string & from, const std::string & to)
{
  plansys2_msgs::msg::Plan plan;
  plansys2_msgs::msg::PlanItem item;
  item.time = 0.0;
  item.action = "(move r2d2 " + from + " " + to + ")";
  item.duration = 1.0;
  plan.items.push_back(item);
  return plan;
}

}  // namespace

class ExecutorLifecycleTest : public ::testing::Test
{
protected:
  // Experts and performer are always active; the executor is left configured or active
  void start(int performer_ticks, bool activate_executor = true, double component_timeout = 30.0)
  {
    std::string pkgpath = plansys2::share_path("plansys2_executor").string();
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    executor_node_ = std::make_shared<plansys2::ExecutorNode>();
    // CTest runs this suite with each BT builder (see CMakeLists.txt)
    if (const char * builder = std::getenv("PLANSYS2_TEST_BT_BUILDER")) {
      executor_node_->set_parameter({"bt_builder_plugin", std::string(builder)});
    }
    executor_node_->set_parameter({"component_timeout", component_timeout});
    move_node_ = std::make_shared<CountingAction>("move_performer", performer_ticks);
    move_node_->set_parameter({"action_name", "move"});
    move_node_->set_parameter({"rate", 20.0});
    domain_node_->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});

    exe_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    for (auto node : nodes()) {
      exe_->add_node(node->get_node_base_interface());
    }
    spinner_ = std::make_unique<ScopedSpinner>(*exe_);

    for (auto node : nodes()) {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    }
    domain_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    problem_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    if (activate_executor) {
      executor_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }

    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(client_node_);
    ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("r2d2", "robot")));
    for (auto zone : {"wheels_zone", "steering_wheels_zone", "assembly_zone"}) {
      ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance(zone, "zone")));
    }
    for (auto pred : {"(robot_at r2d2 wheels_zone)", "(robot_available r2d2)",
        "(battery_full r2d2)"})
    {
      ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate(pred)));
    }
  }

  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes()
  {
    return {domain_node_, problem_node_, executor_node_, move_node_};
  }

  // Stops spinning and drops every node, so they can be checked for destruction
  void stop()
  {
    executor_client_.reset();
    problem_client_.reset();
    spinner_.reset();
    exe_.reset();
    domain_node_.reset();
    problem_node_.reset();
    executor_node_.reset();
    move_node_.reset();
  }

  void TearDown() override {stop();}

  std::optional<ExecutePlan::Result> wait_result(std::chrono::seconds timeout = 30s)
  {
    auto start = std::chrono::steady_clock::now();
    while (executor_client_->execute_and_check_plan()) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return {};
      }
      std::this_thread::sleep_for(20ms);
    }
    return executor_client_->getResult();
  }

  // What the action server answers in any state, to a plain action client (PlanSys2's waits)
  bool server_accepts(const plansys2_msgs::msg::Plan & plan)
  {
    auto client = rclcpp_action::create_client<ExecutePlan>(client_node_, "execute_plan");
    if (!client->wait_for_action_server(5s)) {
      return false;
    }
    ExecutePlan::Goal goal;
    goal.plan = plan;
    auto future = client->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready || !future.get()) {
      return false;
    }
    client->async_cancel_all_goals();
    return true;
  }

  bool robot_at(const std::string & zone)
  {
    return problem_client_->existPredicate(plansys2::Predicate("(robot_at r2d2 " + zone + ")"));
  }

  void expect_plan_succeeds(const std::string & from, const std::string & to)
  {
    ASSERT_TRUE(executor_client_->start_plan_execution(move_plan(from, to)));
    auto result = wait_result();
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
    ASSERT_TRUE(robot_at(to));
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  std::shared_ptr<plansys2::ExecutorNode> executor_node_;
  std::shared_ptr<CountingAction> move_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exe_;
  std::unique_ptr<ScopedSpinner> spinner_;

  // A node that is gone. With the executor stopped, as it may be serving the node right now
  template<typename NodeT>
  void gone(std::shared_ptr<NodeT> & node)
  {
    spinner_.reset();
    exe_->remove_node(node->get_node_base_interface());
    node.reset();
    spinner_ = std::make_unique<ScopedSpinner>(*exe_);
  }
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

// #430: the executor node used to be kept alive by its own action executors
TEST_F(ExecutorLifecycleTest, node_is_destroyed_after_a_plan)
{
  start(3);
  expect_plan_succeeds("wheels_zone", "assembly_zone");

  std::weak_ptr<plansys2::ExecutorNode> weak = executor_node_;
  executor_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  stop();
  ASSERT_TRUE(weak.expired());
}

TEST_F(ExecutorLifecycleTest, node_is_destroyed_while_active)
{
  start(3);
  expect_plan_succeeds("wheels_zone", "assembly_zone");
  std::weak_ptr<plansys2::ExecutorNode> weak = executor_node_;
  // No deactivation: the destructor stops the execution thread itself
  stop();
  ASSERT_TRUE(weak.expired());
}

TEST_F(ExecutorLifecycleTest, goals_rejected_unless_active)
{
  start(3, false);
  ASSERT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_FALSE(server_accepts(move_plan("wheels_zone", "assembly_zone")));

  // The ExecutorClient waits until the executor is active, and then starts the plan
  std::atomic<bool> started {false};
  std::thread caller([&]() {
      started = executor_client_->start_plan_execution(move_plan("wheels_zone", "assembly_zone"));
    });
  std::this_thread::sleep_for(2s);
  EXPECT_FALSE(started.load());
  executor_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  caller.join();
  ASSERT_TRUE(started.load());
  auto result = wait_result();
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
  ASSERT_TRUE(robot_at("assembly_zone"));

  executor_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  ASSERT_FALSE(server_accepts(move_plan("assembly_zone", "wheels_zone")));
  ASSERT_TRUE(robot_at("assembly_zone"));
}

TEST_F(ExecutorLifecycleTest, deactivate_during_a_plan_fails_it)
{
  start(60);  // 3 s per action
  ASSERT_TRUE(executor_client_->start_plan_execution(move_plan("wheels_zone", "assembly_zone")));
  std::this_thread::sleep_for(1s);

  executor_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  ASSERT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  auto result = wait_result(10s);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::FAILURE);

  // The action never completed, even after its 3 s
  std::this_thread::sleep_for(3s);
  ASSERT_EQ(move_node_->finished, 0);
}

// The problem expert goes in mid-plan: the execution thread waits for it, and stopping the
// executor (deactivated or destroyed) must not wait for it too
TEST_F(ExecutorLifecycleTest, stops_while_waiting_for_a_gone_problem_expert)
{
  for (bool destroy : {false, true}) {
    SCOPED_TRACE(destroy ? "destroyed" : "deactivated");
    start(30);  // 1.5 s per action
    ASSERT_TRUE(
      executor_client_->start_plan_execution(move_plan("wheels_zone", "assembly_zone")));
    std::this_thread::sleep_for(500ms);
    // Gone: nobody answers its services any more
    gone(problem_node_);
    // The action ends, and the executor waits for the problem expert to apply its effects
    ASSERT_TRUE(plansys2::testing::wait_for([&]() {return move_node_->finished > 0;}, 5s));
    std::this_thread::sleep_for(2s);

    const auto begin = std::chrono::steady_clock::now();
    if (destroy) {
      std::weak_ptr<plansys2::ExecutorNode> weak = executor_node_;
      stop();
      EXPECT_TRUE(weak.expired());
    } else {
      executor_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
      EXPECT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
    }
    EXPECT_LT(std::chrono::steady_clock::now() - begin, 8s);
    stop();
  }
}

// The problem expert goes in mid-plan, and nobody brings it back: the plan fails once
// component_timeout passes, instead of waiting for it for ever
TEST_F(ExecutorLifecycleTest, a_plan_fails_when_a_component_does_not_come_back)
{
  start(30, true, 2.0);  // 1.5 s per action
  ASSERT_TRUE(executor_client_->start_plan_execution(move_plan("wheels_zone", "assembly_zone")));
  std::this_thread::sleep_for(500ms);
  gone(problem_node_);
  const auto begin = std::chrono::steady_clock::now();
  auto result = wait_result(30s);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->result, ExecutePlan::Result::FAILURE);
  // The action (1 s more), and the timeout of each call that waits for it
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 15s);
  // And the executor goes on: it takes a plan again once there is a problem expert
  EXPECT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(ExecutorLifecycleTest, reactivations_run_a_single_execution_thread)
{
  start(3);
  for (int i = 0; i < 5; i++) {
    executor_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
    executor_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  }

  // With several execution threads the same action would be dealt more than once
  expect_plan_succeeds("wheels_zone", "assembly_zone");
  expect_plan_succeeds("assembly_zone", "steering_wheels_zone");
  std::this_thread::sleep_for(1s);
  ASSERT_EQ(move_node_->finished, 2);
}

TEST_F(ExecutorLifecycleTest, cleanup_and_configure_again)
{
  start(3);
  expect_plan_succeeds("wheels_zone", "assembly_zone");

  executor_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  executor_node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  executor_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  executor_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(executor_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  expect_plan_succeeds("assembly_zone", "steering_wheels_zone");
}

// The plan services read what the execution thread changes on every tick
TEST_F(ExecutorLifecycleTest, plan_services_during_execution)
{
  start(20);  // 1 s per action
  auto query_node = rclcpp::Node::make_shared("plan_query_node");
  auto plan_client = query_node->create_client<plansys2_msgs::srv::GetPlan>("executor/get_plan");
  auto remaining_client =
    query_node->create_client<plansys2_msgs::srv::GetPlan>("executor/get_remaining_plan");
  ASSERT_TRUE(plan_client->wait_for_service(5s));
  ASSERT_TRUE(remaining_client->wait_for_service(5s));

  std::atomic<bool> stop_queries {false};
  std::atomic<int> answers {0};
  std::thread queries([&]() {
      while (!stop_queries) {
        for (auto client : {plan_client, remaining_client}) {
          auto future = client->async_send_request(
            std::make_shared<plansys2_msgs::srv::GetPlan::Request>());
          if (rclcpp::spin_until_future_complete(query_node, future, 1s) ==
          rclcpp::FutureReturnCode::SUCCESS)
          {
            answers++;
          }
        }
      }
    });

  plansys2_msgs::msg::Plan plan = move_plan("wheels_zone", "steering_wheels_zone");
  auto second = move_plan("steering_wheels_zone", "assembly_zone").items[0];
  second.time = 1.001;
  plan.items.push_back(second);
  ASSERT_TRUE(executor_client_->start_plan_execution(plan));
  auto result = wait_result();
  stop_queries = true;
  queries.join();

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->result, ExecutePlan::Result::SUCCESS);
  ASSERT_TRUE(robot_at("assembly_zone"));
  ASSERT_GT(answers, 20);
}

// Ctrl-C with a plan in execution: publishing feedback or finishing the goal after
// rclcpp::shutdown() used to throw from the execution thread and abort the process.
// CTest runs it at several moments of the execution cycle, each in its own process, as
// rclcpp is not meant to be initialized again in the same process (see CMakeLists.txt)
TEST_F(ExecutorLifecycleTest, shutdown_during_a_plan)
{
  const char * delay_ms = std::getenv("PLANSYS2_TEST_SHUTDOWN_DELAY_MS");
  const auto delay = std::chrono::milliseconds(delay_ms ? std::atoi(delay_ms) : 500);

  start(200);  // 10 s per action
  ASSERT_TRUE(executor_client_->start_plan_execution(move_plan("wheels_zone", "assembly_zone")));
  std::this_thread::sleep_for(delay);
  ASSERT_EQ(move_node_->finished, 0);

  rclcpp::shutdown();
  std::this_thread::sleep_for(200ms);
  // Destroys the executor node too: its destructor fails the goal it was running
  stop();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
