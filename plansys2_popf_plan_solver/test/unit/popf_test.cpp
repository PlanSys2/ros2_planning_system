// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

TEST(popf_plan_solver, generate_plan_good)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_popf_plan_solver");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str, "generate_plan_good");

  ASSERT_TRUE(plan);
  ASSERT_EQ(plan.value().size(), 3);
  ASSERT_EQ(plan.value()[0].action, "(move leia kitchen bedroom)");
  ASSERT_EQ(plan.value()[1].action, "(approach leia bedroom jack)");
  ASSERT_EQ(plan.value()[2].action, "(talk leia jack jack m1)");
}
/*
TEST(popf_plan_solver, generate_plan_unsolvable)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_popf_plan_solver");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_2.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str);

  ASSERT_FALSE(plan);
}

TEST(popf_plan_solver, generate_plan_error)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_popf_plan_solver");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_3.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str);

  ASSERT_FALSE(plan);
}*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
