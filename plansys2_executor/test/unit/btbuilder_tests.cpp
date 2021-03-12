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
#include <thread>
#include <fstream>
#include <map>
#include <set>
#include <list>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class BTBuilderTest : public plansys2::BTBuilder
{
public:
  explicit BTBuilderTest(rclcpp::Node::SharedPtr node)
  : BTBuilder(node) {}

  std::string get_tree(const plansys2::Plan & current_plan)
  {
    return BTBuilder::get_tree(current_plan);
  }

  void init_predicates(
    std::set<std::string> & predicates,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
  {
    return BTBuilder::init_predicates(predicates, problem_client);
  }

  void init_functions(
    std::map<std::string, double> & functions,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
  {
    return BTBuilder::init_functions(functions, problem_client);
  }

  std::vector<plansys2::ActionStamped> get_plan_actions(const plansys2::Plan & plan)
  {
    return BTBuilder::get_plan_actions(plan);
  }

  bool is_action_executable(
    const plansys2::ActionStamped & action,
    std::set<std::string> & predicates,
    std::map<std::string, double> & functions) const
  {
    return BTBuilder::is_action_executable(action, predicates, functions);
  }

  plansys2::Graph::Ptr get_graph(const plansys2::Plan & current_plan)
  {
    return BTBuilder::get_graph(current_plan);
  }

  std::list<plansys2::GraphNode::Ptr> get_roots(
    std::vector<plansys2::ActionStamped> & action_sequence,
    std::set<std::string> & predicates,
    std::map<std::string, double> & functions,
    int & node_counter)
  {
    return BTBuilder::get_roots(action_sequence, predicates, functions, node_counter);
  }

  plansys2::GraphNode::Ptr get_node_satisfy(
    const std::shared_ptr<parser::pddl::tree::TreeNode> requirement,
    const std::list<plansys2::GraphNode::Ptr> & roots,
    const plansys2::GraphNode::Ptr & current)
  {
    return BTBuilder::get_node_satisfy(requirement, roots, current);
  }

  plansys2::GraphNode::Ptr get_node_satisfy(
    const std::shared_ptr<parser::pddl::tree::TreeNode> requirement,
    const plansys2::GraphNode::Ptr & node,
    const plansys2::GraphNode::Ptr & current)
  {
    return BTBuilder::get_node_satisfy(requirement, node, current);
  }


  void print_graph(const plansys2::Graph::Ptr & graph) const
  {
    BTBuilder::print_graph(graph);
  }

  void remove_existing_requirements(
    std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> & requirements,
    std::set<std::string> & predicates,
    std::map<std::string, double> & functions) const
  {
    BTBuilder::remove_existing_requirements(requirements, predicates, functions);
  }
};

TEST(btbuilder_tests, test_plan_1)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_1");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);

  auto btbuilder = std::make_shared<BTBuilderTest>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance({"leia", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"entrance", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"kitchen", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"bedroom", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"dinning", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"bathroom", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"chargingroom", "room"}));

  std::vector<std::string> predicates = {
    "(connected entrance dinning)",
    "(connected dinning entrance)",
    "(connected dinning kitchen)",
    "(connected kitchen dinning)",
    "(connected dinning bedroom)",
    "(connected bedroom dinning)",
    "(connected bathroom bedroom)",
    "(connected bedroom bathroom)",
    "(connected chargingroom kitchen)",
    "(connected kitchen chargingroom)",
    "(charging_point_at chargingroom)",
    "(battery_low leia)",
    "(robot_at leia entrance)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        "(and(robot_at leia bathroom))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);


  std::set<std::string> predicates_set;
  btbuilder->init_predicates(predicates_set, problem_client);

  std::map<std::string, double> functions_map;
  btbuilder->init_functions(functions_map, problem_client);

  ASSERT_EQ(problem_client->getPredicates().size(), predicates_set.size());
  ASSERT_EQ(problem_client->getFunctions().size(), functions_map.size());

  auto action_sequence = btbuilder->get_plan_actions(plan.value());

  ASSERT_EQ(action_sequence.size(), 6u);

  ASSERT_NEAR(action_sequence[0].time, 0.000, 0.0001);
  ASSERT_EQ(action_sequence[0].action->name, "askcharge");
  ASSERT_EQ(action_sequence[0].action->parameters[0].name, "leia");
  ASSERT_EQ(action_sequence[0].action->parameters[1].name, "entrance");
  ASSERT_EQ(action_sequence[0].action->parameters[2].name, "chargingroom");
  ASSERT_EQ(
    action_sequence[0].action->at_start_effects.toString(),
    "(and (not (robot_at leia entrance))(robot_at leia chargingroom))");
  std::vector<parser::pddl::tree::Predicate> action_0_predicates;
  action_sequence[0].action->at_start_effects.getPredicates(action_0_predicates);
  ASSERT_EQ(action_0_predicates.size(), 2u);
  ASSERT_EQ(action_0_predicates[0].name, "robot_at");
  ASSERT_EQ(action_0_predicates[0].parameters[0].name, "leia");
  ASSERT_EQ(action_0_predicates[0].parameters[1].name, "entrance");
  ASSERT_TRUE(action_0_predicates[0].negative);

  ASSERT_TRUE(
    plansys2::check(
      action_sequence[0].action->at_start_requirements.root_,
      problem_client));

  ASSERT_FALSE(
    plansys2::check(
      action_sequence[1].action->at_start_requirements.root_,
      problem_client));
  ASSERT_FALSE(
    plansys2::check(
      action_sequence[2].action->at_start_requirements.root_,
      problem_client));
  ASSERT_FALSE(
    plansys2::check(
      action_sequence[3].action->at_start_requirements.root_,
      problem_client));
  ASSERT_FALSE(
    plansys2::check(
      action_sequence[4].action->at_start_requirements.root_,
      problem_client));
  ASSERT_FALSE(
    plansys2::check(
      action_sequence[5].action->at_start_requirements.root_,
      problem_client));

  ASSERT_TRUE(btbuilder->is_action_executable(action_sequence[0], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[1], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[2], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[3], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[4], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[5], predicates_set, functions_map));

  ASSERT_NE(predicates_set.find("(robot_at leia entrance)"), predicates_set.end());
  ASSERT_NE(predicates_set.find("(robot_at leia entrance)"), predicates_set.end());
  ASSERT_EQ(
    predicates_set.find("(robot_at leia chargingroom)"),
    predicates_set.end());

  plansys2::apply(
    action_sequence[0].action->at_start_effects.root_,
    predicates_set, functions_map);
  plansys2::apply(
    action_sequence[0].action->at_end_effects.root_,
    predicates_set, functions_map);

  ASSERT_EQ(predicates_set.find("(robot_at leia entrance)"), predicates_set.end());
  ASSERT_NE(
    predicates_set.find("(robot_at leia chargingroom)"),
    predicates_set.end());

  ASSERT_TRUE(btbuilder->is_action_executable(action_sequence[1], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[2], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[3], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[4], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[5], predicates_set, functions_map));
  plansys2::apply(
    action_sequence[1].action->at_start_effects.root_,
    predicates_set, functions_map);
  plansys2::apply(
    action_sequence[1].action->at_end_effects.root_,
    predicates_set, functions_map);

  ASSERT_TRUE(btbuilder->is_action_executable(action_sequence[2], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[3], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[4], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[5], predicates_set, functions_map));
  plansys2::apply(
    action_sequence[2].action->at_start_effects.root_,
    predicates_set, functions_map);
  plansys2::apply(
    action_sequence[2].action->at_end_effects.root_,
    predicates_set, functions_map);

  ASSERT_TRUE(btbuilder->is_action_executable(action_sequence[3], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[4], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[5], predicates_set, functions_map));
  plansys2::apply(
    action_sequence[3].action->at_start_effects.root_,
    predicates_set, functions_map);
  plansys2::apply(
    action_sequence[3].action->at_end_effects.root_,
    predicates_set, functions_map);

  ASSERT_TRUE(btbuilder->is_action_executable(action_sequence[4], predicates_set, functions_map));
  ASSERT_FALSE(btbuilder->is_action_executable(action_sequence[5], predicates_set, functions_map));
  plansys2::apply(
    action_sequence[4].action->at_start_effects.root_,
    predicates_set, functions_map);
  plansys2::apply(
    action_sequence[4].action->at_end_effects.root_,
    predicates_set, functions_map);

  ASSERT_TRUE(btbuilder->is_action_executable(action_sequence[5], predicates_set, functions_map));
  plansys2::apply(
    action_sequence[5].action->at_start_effects.root_,
    predicates_set, functions_map);
  plansys2::apply(
    action_sequence[5].action->at_end_effects.root_,
    predicates_set, functions_map);

  ASSERT_NE(predicates_set.find("(robot_at leia bathroom)"), predicates_set.end());

  finish = true;
  t.join();
}


TEST(btbuilder_tests, test_plan_2)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_2");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);

  auto btbuilder = std::make_shared<BTBuilderTest>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance({"robot1", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"robot2", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"robot3", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"wheels_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"steering_wheels_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"body_car_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"assembly_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"wheel_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"wheel_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"wheel_3", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"body_car_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"body_car_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"body_car_3", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"steering_wheel_1", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"steering_wheel_2", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"steering_wheel_3", "piece"}));
  ASSERT_TRUE(problem_client->addInstance({"car_1", "car"}));
  ASSERT_TRUE(problem_client->addInstance({"car_2", "car"}));
  ASSERT_TRUE(problem_client->addInstance({"car_3", "car"}));

  std::vector<std::string> predicates = {
    "(robot_at robot1 assembly_zone)",
    "(robot_at robot2 assembly_zone)",
    "(robot_at robot3 assembly_zone)",
    "(is_assembly_zone assembly_zone)",
    "(robot_available robot1)",
    "(robot_available robot2)",
    "(robot_available robot3)",
    "(piece_at wheel_1 wheels_zone)",
    "(piece_at body_car_1 body_car_zone)",
    "(piece_at steering_wheel_1 steering_wheels_zone)",
    "(piece_is_wheel wheel_1)",
    "(piece_is_body_car body_car_1)",
    "(piece_is_steering_wheel steering_wheel_1)",
    "(piece_at wheel_2 wheels_zone)",
    "(piece_at body_car_2 body_car_zone)",
    "(piece_at steering_wheel_2 steering_wheels_zone)",
    "(piece_is_wheel wheel_2)",
    "(piece_is_body_car body_car_2)",
    "(piece_is_steering_wheel steering_wheel_2)",
    "(piece_at wheel_3 wheels_zone)",
    "(piece_at body_car_3 body_car_zone)",
    "(piece_at steering_wheel_3 steering_wheels_zone)",
    "(piece_is_wheel wheel_3)",
    "(piece_is_body_car body_car_3)",
    "(piece_is_steering_wheel steering_wheel_3)",
    "(piece_not_used wheel_1)",
    "(piece_not_used wheel_2)",
    "(piece_not_used wheel_3)",
    "(piece_not_used body_car_1)",
    "(piece_not_used body_car_2)",
    "(piece_not_used body_car_3)",
    "(piece_not_used steering_wheel_1)",
    "(piece_not_used steering_wheel_2)",
    "(piece_not_used steering_wheel_3)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }


  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        "(and(car_assembled car_1)(car_assembled car_2)(car_assembled car_3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::set<std::string> predicates_set;
  btbuilder->init_predicates(predicates_set, problem_client);

  std::map<std::string, double> functions_map;
  btbuilder->init_functions(functions_map, problem_client);


  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> check_predicates;
  for (const auto & pred : predicates) {
    std::shared_ptr<parser::pddl::tree::PredicateNode> pred_node(
      new parser::pddl::tree::PredicateNode);
    pred_node->predicate_.fromString(pred);
    std::shared_ptr<parser::pddl::tree::TreeNode> tree_node =
      std::dynamic_pointer_cast<parser::pddl::tree::TreeNode>(pred_node);
    check_predicates.push_back(tree_node);
  }
  std::shared_ptr<parser::pddl::tree::PredicateNode> pred_node(
    new parser::pddl::tree::PredicateNode);
  pred_node->predicate_.fromString("(is_assembly_zone body_car_zone)");
  std::shared_ptr<parser::pddl::tree::TreeNode> tree_node =
    std::dynamic_pointer_cast<parser::pddl::tree::TreeNode>(pred_node);
  check_predicates.push_back(tree_node);

  btbuilder->remove_existing_requirements(check_predicates, predicates_set, functions_map);

  ASSERT_EQ(check_predicates.size(), 1);
  pred_node.reset(new parser::pddl::tree::PredicateNode);
  pred_node =
    std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(*check_predicates.begin());
  ASSERT_EQ(pred_node->toString(), "(is_assembly_zone body_car_zone)");


  ASSERT_EQ(problem_client->getPredicates().size(), predicates_set.size());

  auto action_sequence = btbuilder->get_plan_actions(plan.value());

  ASSERT_EQ(action_sequence.size(), 22u);

  int node_counter = 0;
  auto roots = btbuilder->get_roots(action_sequence, predicates_set, functions_map, node_counter);
  ASSERT_EQ(roots.size(), 3u);
  // Apply roots actions
  for (auto & action_node : roots) {
    btbuilder->init_predicates(action_node->predicates, problem_client);
    btbuilder->init_functions(action_node->functions, problem_client);
    plansys2::apply(
      action_node->action.action->at_start_effects.root_,
      action_node->predicates, action_node->functions);
    plansys2::apply(
      action_node->action.action->at_end_effects.root_,
      action_node->predicates, action_node->functions);
    plansys2::apply(
      action_node->action.action->at_start_effects.root_,
      predicates_set, functions_map);
    plansys2::apply(
      action_node->action.action->at_end_effects.root_,
      predicates_set, functions_map);
  }

  ASSERT_NE(
    predicates_set.find("(robot_at robot1 body_car_zone)"),
    predicates_set.end());
  ASSERT_NE(
    predicates_set.find("(robot_at robot2 steering_wheels_zone)"),
    predicates_set.end());
  ASSERT_NE(
    predicates_set.find("(robot_at robot3 wheels_zone)"),
    predicates_set.end());

  pred_node.reset(new parser::pddl::tree::PredicateNode);
  pred_node->predicate_.fromString("(robot_at robot1 body_car_zone)");
  std::shared_ptr<parser::pddl::tree::TreeNode> test_pred_1 =
    std::dynamic_pointer_cast<parser::pddl::tree::TreeNode>(pred_node);
  auto node_satisfy_1 = btbuilder->get_node_satisfy(test_pred_1, *roots.begin(), nullptr);
  ASSERT_NE(node_satisfy_1, nullptr);
  ASSERT_EQ(node_satisfy_1->action.action->name, "move");
  ASSERT_EQ(node_satisfy_1->action.action->parameters.size(), 3u);
  ASSERT_EQ(node_satisfy_1->action.action->parameters[0].name, "robot1");
  ASSERT_EQ(node_satisfy_1->action.action->parameters[1].name, "assembly_zone");
  ASSERT_EQ(node_satisfy_1->action.action->parameters[2].name, "body_car_zone");

  auto it = roots.begin();
  it++;
  ASSERT_EQ(btbuilder->get_node_satisfy(test_pred_1, *it, nullptr), nullptr);
  it++;
  ASSERT_EQ(btbuilder->get_node_satisfy(test_pred_1, *it, nullptr), nullptr);

  auto graph = btbuilder->get_graph(plan.value());
  ASSERT_NE(graph, nullptr);

  btbuilder->print_graph(graph);

  finish = true;
  t.join();
}

TEST(btbuilder_tests, test_plan_3)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_3");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);

  auto btbuilder = std::make_shared<BTBuilderTest>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance({"leia", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"ro1", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"ro2", "room"}));
  ASSERT_TRUE(problem_client->addInstance({"ro3", "room"}));

  std::vector<std::string> predicates = {
    "(connected ro1 ro2)",
    "(connected ro2 ro1)",
    "(connected ro1 ro3)",
    "(connected ro3 ro1)",
    "(connected ro2 ro3)",
    "(connected ro3 ro2)",
    "(robot_at leia ro1)",
    "(battery_full leia)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        "(and (patrolled ro1) (patrolled ro2) (patrolled ro3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto bt = btbuilder->get_tree(plan.value());

  std::cerr << bt << std::endl;


  finish = true;
  t.join();
}

TEST(btbuilder_tests, test_plan_4)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_4");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);

  auto btbuilder = std::make_shared<BTBuilderTest>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/cooking_domain.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/cooking_domain.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance({"r2d2", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"c3po", "robot"}));
  ASSERT_TRUE(problem_client->addInstance({"fridge_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"pantry_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"watertap_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"cooking_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"recharge_zone", "zone"}));
  ASSERT_TRUE(problem_client->addInstance({"eggs", "ingredient"}));
  ASSERT_TRUE(problem_client->addInstance({"oil", "ingredient"}));
  ASSERT_TRUE(problem_client->addInstance({"salt", "ingredient"}));
  ASSERT_TRUE(problem_client->addInstance({"omelette", "dish"}));
  ASSERT_TRUE(problem_client->addInstance({"flour", "ingredient"}));
  ASSERT_TRUE(problem_client->addInstance({"sugar", "ingredient"}));
  ASSERT_TRUE(problem_client->addInstance({"cake", "dish"}));

  std::vector<std::string> predicates = {
    "(is_cooking_zone cooking_zone)",
    "(is_fridge_zone fridge_zone)",
    "(is_fridge_zone fridge_zone)",
    "(is_watertap_zone watertap_zone)",
    "(is_recharge_zone recharge_zone)",
    "(ingredient_at eggs fridge_zone)",
    "(ingredient_at oil pantry_zone)",
    "(ingredient_at salt pantry_zone)",
    "(ingredient_at flour pantry_zone)",
    "(ingredient_at sugar pantry_zone)",
    "(is_oil oil)",
    "(is_egg eggs)",
    "(is_salt salt)",
    "(is_flour flour)",
    "(is_sugar sugar)",
    "(is_cake cake)",
    "(is_omelette omelette)",
    "(robot_at r2d2 cooking_zone)",
    "(battery_full r2d2)",
    "(robot_at c3po cooking_zone)",
    "(battery_full c3po)"
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(parser::pddl::tree::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      parser::pddl::tree::Goal(
        "(and (dish_prepared cake)(dish_prepared omelette))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  btbuilder->print_graph(btbuilder->get_graph(plan.value()));
  auto bt = btbuilder->get_tree(plan.value());

  std::cerr << bt << std::endl;

  // Todo: Test that the BT is correct

  finish = true;
  t.join();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
