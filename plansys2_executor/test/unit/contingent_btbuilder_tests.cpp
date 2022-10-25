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
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_blocks_observe.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_blocks_observe.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 4);
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  bool finish = false;
  std::thread t([&]() {
    while (!finish) { exe.spin_some(); }
  });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_blocks_observe.pddl");
  std::string problem_str((std::istreambuf_iterator<char>(problem_ifs)),
                          std::istreambuf_iterator<char>());
  problem_client->addProblem(problem_str);

  pluginlib::ClassLoader<plansys2::PlanSolverBase> plan_loader("plansys2_core", "plansys2::PlanSolverBase");
  std::optional<plansys2_msgs::msg::Plan> plan;
  auto planner = plan_loader.createUniqueInstance("plansys2/CFFPlanSolver");
  plan = planner->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  pluginlib::ClassLoader<plansys2::BTBuilder> btbuilder_loader("plansys2_executor", "plansys2::BTBuilder");
  auto btbuilder = btbuilder_loader.createUniqueInstance("plansys2::ContingentBTBuilder");
  btbuilder->initialize();
  auto tree = btbuilder->get_tree(plan.value());
  std::cout << tree << std::endl;
  std::ifstream expected_tree_ifs(pkgpath + "/test_behavior_trees/test_contingent_bt.xml");
  std::string expected_tree((std::istreambuf_iterator<char>(expected_tree_ifs)),
                            std::istreambuf_iterator<char>());

  ASSERT_EQ(expected_tree, tree);

  finish = true;
  t.join();

}

//TODO need to add graphviz test

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
