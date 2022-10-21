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
#include <regex>
#include <iostream>
#include <memory>
#include <fstream>
#include <map>
#include <set>
#include <list>
#include <tuple>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/bt_builder_plugins/contingent_bt_builder.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


TEST(simple_btbuilder_tests, test_tree_builder) {

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  std::ifstream domain_ifs(pkgpath + "/pddl/domain_blocks_observe.pddl");
  std::string domain_str((std::istreambuf_iterator<char>(domain_ifs)),
                           std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_blocks_observe.pddl");
  std::string problem_str((std::istreambuf_iterator<char>(problem_ifs)),
                          std::istreambuf_iterator<char>());


  auto domain = std::make_shared<plansys2::DomainExpert>(domain_str);
  auto problem = std::make_shared<plansys2::ProblemExpert>(domain);
  problem->addProblem(problem_str);

  pluginlib::ClassLoader<plansys2::PlanSolverBase> plan_loader("plansys2_core", "plansys2::PlanSolverBase");
  auto planner = plan_loader.createUniqueInstance("plansys2/CFFPlanSolver");

  auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  ASSERT_TRUE(plan);

  auto btbuilder = std::make_shared<plansys2::ContingentBTBuilder>();

  auto tree = btbuilder->get_tree(plan.value());
  std::cout << tree << std::endl;

}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
