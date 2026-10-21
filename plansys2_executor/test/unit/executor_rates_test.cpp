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

// What the executor publishes while a plan runs: at feedback_rate, not on every tick

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;

namespace
{
// Each move takes about a second
class Move : public plansys2::ActionExecutorClient
{
public:
  Move()
  : ActionExecutorClient("rates_move")
  {
    set_parameter({"action_name", "move"});
    set_parameter({"rate", 20.0});
  }

private:
  void do_work() override
  {
    if (++ticks_ >= 20) {
      ticks_ = 0;
      finish(true, 1.0, "done");
    } else {
      send_feedback(ticks_ / 20.0, "moving");
    }
  }
  int ticks_ {0};
};

struct Counts
{
  int remaining_plans = 0;
  int executing_plans = 0;
  int dotgraphs = 0;
  double seconds = 0.0;
};

Counts run_plan(double feedback_rate, bool listen_to_dotgraph)
{
  Counts counts;
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();
  auto move = std::make_shared<Move>();
  auto listener = rclcpp::Node::make_shared("rates_listener");

  std::atomic<int> remaining {0}, executing {0}, dotgraphs {0};
  auto remaining_sub = listener->create_subscription<plansys2_msgs::msg::Plan>(
    "remaining_plan", 100, [&](plansys2_msgs::msg::Plan::SharedPtr) {remaining++;});
  auto executing_sub = listener->create_subscription<plansys2_msgs::msg::Plan>(
    "executing_plan", rclcpp::QoS(100).transient_local(),
    [&](plansys2_msgs::msg::Plan::SharedPtr) {executing++;});
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dotgraph_sub;
  if (listen_to_dotgraph) {
    dotgraph_sub = listener->create_subscription<std_msgs::msg::String>(
      "dot_graph", 100, [&](std_msgs::msg::String::SharedPtr) {dotgraphs++;});
  }

  const auto pkgpath = plansys2::share_path("plansys2_executor").string();
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/simple_move_example.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/simple_move_example.pddl"});
  executor_node->set_parameter({"feedback_rate", feedback_rate});

  rclcpp::executors::MultiThreadedExecutor exe;
  for (auto node : {domain_node->get_node_base_interface(), problem_node->get_node_base_interface(),
      planner_node->get_node_base_interface(), executor_node->get_node_base_interface(),
      move->get_node_base_interface(), listener->get_node_base_interface()})
  {
    exe.add_node(node);
  }
  plansys2::testing::ExecutorThread spinner(exe);

  for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
    domain_node->trigger_transition(transition);
    problem_node->trigger_transition(transition);
    planner_node->trigger_transition(transition);
    executor_node->trigger_transition(transition);
  }
  move->trigger_transition(Transition::TRANSITION_CONFIGURE);

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(listener);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(listener);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(listener);
  auto executor_client = std::make_shared<plansys2::ExecutorClient>(listener);

  EXPECT_TRUE(problem_client->addInstance(plansys2::Instance{"r2d2", "robot"}));
  for (const auto & wp : {"wp1", "wp2", "wp3"}) {
    EXPECT_TRUE(problem_client->addInstance(plansys2::Instance{wp, "waypoint"}));
  }
  for (const auto & fact : {"(connected wp1 wp2)", "(connected wp2 wp3)", "(robot_at r2d2 wp1)"}) {
    EXPECT_TRUE(problem_client->addPredicate(plansys2::Predicate(fact)));
  }
  EXPECT_TRUE(problem_client->setGoal(plansys2::Goal("(and (robot_at r2d2 wp3))")));
  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  EXPECT_TRUE(plan.has_value());
  if (!plan) {
    return counts;
  }
  EXPECT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        return remaining_sub->get_publisher_count() > 0 &&
               executing_sub->get_publisher_count() > 0;
      }));

  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(executor_client->start_plan_execution(plan.value()));
  EXPECT_TRUE(
    plansys2::testing::wait_for([&]() {return !executor_client->execute_and_check_plan();}, 30s));
  counts.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_EQ(executor_client->getResult()->result,
      plansys2_msgs::action::ExecutePlan::Result::SUCCESS);
  std::this_thread::sleep_for(200ms);  // The last messages arrive

  counts.remaining_plans = remaining;
  counts.executing_plans = executing;
  counts.dotgraphs = dotgraphs;
  return counts;
}
}  // namespace

TEST(executor_rates, feedback_at_its_rate)
{
  const auto counts = run_plan(5.0, true);
  ASSERT_GT(counts.seconds, 1.0);
  // The tree ticks at 50 Hz: before, all of it was published on every tick
  EXPECT_LE(counts.remaining_plans, 5.0 * counts.seconds + 3);
  EXPECT_GE(counts.remaining_plans, 2);
  EXPECT_LE(counts.dotgraphs, 5.0 * counts.seconds + 3);
  EXPECT_GE(counts.dotgraphs, 1);
  EXPECT_EQ(counts.executing_plans, 1);  // Latched: once per plan
  plansys2::drain_ros();
}

TEST(executor_rates, feedback_rate_is_a_parameter)
{
  const auto counts = run_plan(20.0, false);
  ASSERT_GT(counts.seconds, 1.0);
  EXPECT_GE(counts.remaining_plans, 10.0 * counts.seconds);
  EXPECT_LE(counts.remaining_plans, 20.0 * counts.seconds + 3);
  EXPECT_EQ(counts.dotgraphs, 0);
  plansys2::drain_ros();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
