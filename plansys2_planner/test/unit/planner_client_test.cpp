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

// Its own binary: no plan_solver_timeout given to every node, as a launch with a params file

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace
{

// Takes `takes` to find a one-action plan, whatever its timeout or a cancel say
class SlowSolver : public plansys2::PlanSolverBase
{
public:
  explicit SlowSolver(std::chrono::milliseconds takes)
  : takes_(takes) {}

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, const std::string &) override {}

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string &, const std::string &, const std::string &,
    const rclcpp::Duration) override
  {
    std::this_thread::sleep_for(takes_);
    plansys2_msgs::msg::Plan plan;
    plan.items.emplace_back().action = "(slow)";
    return plan;
  }

  bool isDomainValid(const std::string &, const std::string &) override {return true;}

private:
  std::chrono::milliseconds takes_;
};

class TestPlannerNode : public plansys2::PlannerNode
{
public:
  void set_only_solver(plansys2::PlanSolverBase::Ptr solver)
  {
    solvers_.clear();
    solvers_["SLOW"] = solver;
  }
};

}  // namespace

// The client waited 15 s and a margin, whatever the planner was set to: with the planner
// planning longer, the client gave up before its answer. It waits as the planner plans
TEST(PlannerClient, waits_as_long_as_the_planner_plans)
{
  auto planner = std::make_shared<TestPlannerNode>();
  planner->set_parameter({"plan_solver_timeout", 17.0});
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(planner->get_node_base_interface());
  plansys2::testing::ExecutorThread spinner(exe);

  planner->trigger_transition(Transition::TRANSITION_CONFIGURE);
  planner->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(planner->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  // After the client's old wait (15 + 5 s), before the new one (17 + 5 s)
  planner->set_only_solver(std::make_shared<SlowSolver>(21s));

  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto client = std::make_shared<plansys2::PlannerClient>(client_node);
  const auto start = std::chrono::steady_clock::now();
  auto plan = client->getPlan(
    "(define (domain d) (:requirements :strips) (:predicates (p)))",
    "(define (problem p) (:domain d) (:init) (:goal (and (p))))");
  ASSERT_TRUE(plan.has_value());
  ASSERT_EQ(plan->items.size(), 1u);
  EXPECT_EQ(plan->items[0].action, "(slow)");
  EXPECT_GT(std::chrono::steady_clock::now() - start, 20s);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
