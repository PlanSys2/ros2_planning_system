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

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"

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
  BTBuilderTest(rclcpp::Node::SharedPtr node) : BTBuilder(node) {}

  std::string get_tree(const plansys2::Plan & current_plan)
  {
    return BTBuilder::get_tree(current_plan);
  }

  std::vector<plansys2::ExecutionLevel::Ptr> get_plan_actions(const plansys2::Plan & plan)
  {
    return BTBuilder::get_plan_actions(plan);
  }

  void print_levels(std::vector<plansys2::ExecutionLevel::Ptr> & levels) {
    BTBuilder::print_levels(levels);
  }

  void check_connections(
    plansys2::ExecutionLevel::Ptr up_level,
    plansys2::ExecutionLevel::Ptr down_level)
  {
    BTBuilder::check_connections(up_level, down_level);
  }

  bool level_satisfied(plansys2::ExecutionLevel::Ptr level)
  {
    return BTBuilder::level_satisfied(level);
  }

  int in_cardinality(plansys2::ActionUnit::Ptr action_unit)
  {
    return BTBuilder::in_cardinality(action_unit);
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
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal(
    "(and(robot_at leia bathroom))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  /*
  std::vector<plansys2::ExecutionLevel::Ptr> levels = btbuilder->get_plan_actions(plan.value());

  /*ASSERT_EQ(levels.size(), 6u);
  ASSERT_EQ(levels[0]->action_units.size(), 1u);

  auto it = levels[0]->action_units.begin();
  ASSERT_EQ((*it++)->action, "(askcharge leia entrance chargingroom)");
  ASSERT_EQ((*it)->action, "(charge leia chargingroom)");
  ASSERT_EQ(levels[1]->action_units.size(), 1u);
  ASSERT_EQ(levels[2]->action_units.size(), 1u);
  ASSERT_EQ(levels[3]->action_units.size(), 1u);
  ASSERT_EQ(levels[4]->action_units.size(), 1u);
  
  for (int i = 1; i < levels.size(); i++) {
    int level_comp = i - 1;
    while (level_comp >= 0 && !btbuilder->level_satisfied(levels[i])) {
      
      std::cerr << "[" << level_comp << " - " << i << "] ======================" << std::endl;
      btbuilder->check_connections(levels[level_comp], levels[i]);
      level_comp--;
    }
  }

  for (auto & level : levels) {
    for (auto & action_unit : level->action_units) {
      for (auto & req : action_unit->at_start_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client->existPredicate(plansys2::Predicate(req->requirement));
        }
      }
      for (auto & req : action_unit->over_all_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client->existPredicate(plansys2::Predicate(req->requirement));
        }
      }
      for (auto & req : action_unit->at_end_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client->existPredicate(plansys2::Predicate(req->requirement));
        }
      }
    }
  }

  int root_counters = 0;
  for (auto & level : levels) {
    for (auto & action_unit : level->action_units) {
      if (btbuilder->in_cardinality(action_unit) == 0) {
        root_counters++;
      }
    }
  }

  std::cerr << "==============> Roots = " << root_counters << std::endl;

  btbuilder->print_levels(levels);


  std::string bt_plan;

  if (root_counters > 1) {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n" +
      t(2) + "<Parallel success_threshold=\"" + std::to_string(root_counters) +
      "\" failure_threshold=\"1\">\n";

    for (auto & level : levels) {
      for (auto & action_unit : level->action_units) {
        if (in_cardinality(action_unit) == 0) {
          std::set<ActionUnit::Ptr> used_actions;
          bt_plan = bt_plan + get_flow_tree(action_unit, used_actions, 3);
        }
      }
    }

    bt_plan = bt_plan + t(2) + "</Parallel>\n" +
      t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n";

    for (auto & level : levels) {
      for (auto & action_unit : level->action_units) {
        if (in_cardinality(action_unit) == 0) {
          std::set<ActionUnit::Ptr> used_actions;
          bt_plan = bt_plan + get_flow_tree(action_unit, used_actions, 2);
        }
      }
    }

    bt_plan = bt_plan + t(1) + "</BehaviorTree>\n</root>\n";
  }

  //std::map<std::string, plansys2::DurativeAction> durative_actions_map;
  */
  
  auto tree_str = btbuilder->get_tree(plan.value());
  std::cerr << tree_str << std::endl;

  finish = true;
  t.join();
}

/*
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
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal(
    "(and (patrolled ro1) (patrolled ro2) (patrolled ro3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::vector<plansys2::ExecutionLevel::Ptr> levels = get_plan_actions(plan)
  print_levels(levels);



  //std::map<std::string, plansys2::DurativeAction> durative_actions_map;
  
  
  // auto tree_str = btbuilder.get_tree(plan.value());
  // std::cerr << tree_str << std::endl;

  finish = true;
  t.join();
}*/


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
