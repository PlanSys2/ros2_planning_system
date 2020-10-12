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
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "rclcpp/rclcpp.hpp"


class BTBuilderTest : public plansys2::BTBuilder
{
public:
  explicit BTBuilderTest(
    rclcpp::Node::SharedPtr node)
  : BTBuilder(node)
  {
  }
};


TEST(executiotest_noden_tree, bt_builder_factory)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);

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
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal(
        "(and(car_assembled car_1)(car_assembled car_2)(car_assembled car_3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::map<std::string, plansys2::DurativeAction> durative_actions_map;
  BTBuilderTest exec_tree(test_node);
  auto tree_str = exec_tree.get_tree(plan.value());

  std::cout << tree_str << std::endl;
  finish = true;
  t.join();
}


TEST(executiotest_noden_tree, bt_builder_factory_2)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);

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
    "(robot_at robot1 wheels_zone)",
    "(robot_at robot2 body_car_zone)",
    "(robot_at robot3 steering_wheels_zone)",
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
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal(
        std::string("(and(car_assembled car_1)(piece_at body_car_2 assembly_zone)") +
        std::string("(piece_at body_car_3 assembly_zone))"))));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::map<std::string, plansys2::DurativeAction> durative_actions_map;
  BTBuilderTest exec_tree(test_node);


  // ASSERT_NE(durative_actions_map.find(
  //   "(move robot1 assembly_zone wheels_zone)"),
  //   durative_actions_map.end());

  auto tree_str = exec_tree.get_tree(plan.value());

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
