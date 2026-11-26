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

#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_core/PackagePaths.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_klimax_plan_solver/klimax_plan_solver.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

std::string read_hddl(const std::string & name)
{
  const auto path = plansys2::share_path("plansys2_klimax_plan_solver") / "hddl" / name;
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

plansys2::PlanSolverBase::Ptr make_solver(const std::string & node_name)
{
  auto solver = std::make_shared<plansys2::KlimaxPlanSolver>();
  solver->configure(rclcpp_lifecycle::LifecycleNode::make_shared(node_name), "KLIMAX");
  return solver;
}

}  // namespace

TEST(klimax_plan_solver, load_plugin_and_plan)
{
  pluginlib::ClassLoader<plansys2::PlanSolverBase> loader(
    "plansys2_core", "plansys2::PlanSolverBase");
  plansys2::PlanSolverBase::Ptr plugin;
  ASSERT_NO_THROW(plugin = loader.createUniqueInstance("plansys2/KlimaxPlanSolver"));
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("klimax_plugin_node");
  plugin->configure(node, "KLIMAX");
  EXPECT_TRUE(node->has_parameter("KLIMAX.timeout"));
  EXPECT_TRUE(node->has_parameter("KLIMAX.anytime"));

  auto plan = plugin->getPlan(read_hddl("house.hddl"), read_hddl("house_problem.hddl"));
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->items.size(), 11u);
  // Sequential: one action after another
  for (size_t i = 0; i < plan->items.size(); ++i) {
    EXPECT_DOUBLE_EQ(plan->items[i].time, static_cast<double>(i));
  }
  plugin.reset();
}

TEST(klimax_plan_solver, the_plan_has_its_decomposition)
{
  auto plan = make_solver("klimax_tree_node")->getPlan(
    read_hddl("house.hddl"), read_hddl("house_problem.hddl"));
  ASSERT_TRUE(plan);
  ASSERT_FALSE(plan->tasks.empty());
  // The initial task network first: two deliveries in no order
  const auto & root = plan->tasks[0];
  EXPECT_EQ(root.id, "root");
  EXPECT_TRUE(root.task.empty());
  EXPECT_EQ(root.subtasks.size(), 2u);
  EXPECT_TRUE(root.ordering.empty());
  // Every subtask is a task of the plan or one of its actions, once
  std::map<std::string, const plansys2_msgs::msg::PlanTask *> by_id;
  for (const auto & task : plan->tasks) {
    by_id[task.id] = &task;
  }
  std::set<std::string> seen;
  for (const auto & task : plan->tasks) {
    for (const auto & sub : task.subtasks) {
      EXPECT_TRUE(seen.insert(sub).second) << sub;
      if (sub.rfind("item:", 0) == 0) {
        EXPECT_LT(std::stoul(sub.substr(5)), plan->items.size()) << sub;
      } else {
        EXPECT_TRUE(by_id.count(sub)) << sub;
      }
    }
  }
  EXPECT_EQ(seen.size(), plan->tasks.size() - 1 + plan->items.size());
  // A delivery: its method, its four subtasks in a row and its precondition with its values
  const auto & deliver = *by_id.at(root.subtasks[0]);
  EXPECT_EQ(deliver.task.rfind("(deliver ", 0), 0u) << deliver.task;
  EXPECT_EQ(deliver.method, "deliver_item");
  EXPECT_EQ(deliver.subtasks.size(), 4u);
  EXPECT_EQ(deliver.ordering, (std::vector<uint32_t>{0, 1, 1, 2, 2, 3}));
  EXPECT_NE(deliver.precondition.find("(free tiago)"), std::string::npos) << deliver.precondition;
  EXPECT_EQ(deliver.precondition.find("?"), std::string::npos) << deliver.precondition;
}

TEST(klimax_plan_solver, what_it_plans_with)
{
  auto solver = make_solver("klimax_requirements_node");
  EXPECT_TRUE(solver->supported_requirements().count(":hierarchy"));
  // A PDDL problem is not its: no plan
  EXPECT_FALSE(
    solver->getPlan(
      "(define (domain d) (:requirements :strips) (:predicates (p))"
      " (:action a :parameters () :effect (p)))",
      "(define (problem x) (:domain d) (:init) (:goal (p)))"));
  // No way to deliver: no plan
  std::string problem = read_hddl("house_problem.hddl");
  problem.replace(problem.find("(free tiago)"), 12, "");
  EXPECT_FALSE(solver->getPlan(read_hddl("house.hddl"), problem, "", 5s));
}

TEST(klimax_plan_solver, check_domains)
{
  auto solver = make_solver("klimax_check_node");
  EXPECT_TRUE(solver->isDomainValid(read_hddl("house.hddl")));
  std::string broken = read_hddl("house.hddl");
  broken.replace(broken.find(":task (deliver ?i ?to)"), 22, ":task (deliver ?i)");
  EXPECT_FALSE(solver->isDomainValid(broken));
  EXPECT_FALSE(solver->isDomainValid(""));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
