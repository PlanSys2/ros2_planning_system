// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>
#include <string>

#include "plansys2_core/PackagePaths.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "plansys2_tests/test_action_node.hpp"
#include "plansys2_tests/execution_logger.hpp"

TEST(test_2, test_2)
{
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(client_node);
  auto executor_client = std::make_shared<plansys2::ExecutorClient>(client_node);

  auto move_action_node = plansys2_tests::TestAction::make_shared("move");
  auto ask_charge_node = plansys2_tests::TestAction::make_shared("askcharge");
  auto charge_node = plansys2_tests::TestAction::make_shared("charge");

  auto execution_logger = plansys2_tests::ExecutionLogger::make_shared();

  std::string pkgpath = plansys2::share_path("plansys2_tests").string();

  domain_node->set_parameter({"model_file", pkgpath + "/test_2/pddl/test_2.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_2/pddl/test_2.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(ask_charge_node->get_node_base_interface());
  exe.add_node(charge_node->get_node_base_interface());
  exe.add_node(execution_logger->get_node_base_interface());

  plansys2::testing::ExecutorThread spinner(exe);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  problem_client->addInstance(plansys2::Instance("leia", "robot"));
  problem_client->addInstance(plansys2::Instance("room_1", "room"));
  problem_client->addInstance(plansys2::Instance("room_2", "room"));
  problem_client->addInstance(plansys2::Instance("corridor_1", "corridor"));
  problem_client->addInstance(plansys2::Instance("zone_1_1", "zone"));
  problem_client->addInstance(plansys2::Instance("zone_1_2", "zone"));
  problem_client->addInstance(plansys2::Instance("zone_recharge", "zone"));
  problem_client->addPredicate(plansys2::Predicate("(connected room_1 corridor_1)"));
  problem_client->addPredicate(plansys2::Predicate("(connected corridor_1 room_1)"));
  problem_client->addPredicate(plansys2::Predicate("(connected room_2 corridor_1)"));
  problem_client->addPredicate(plansys2::Predicate("(connected corridor_1 room_2)"));
  problem_client->addPredicate(plansys2::Predicate("(connected room_1 zone_1_1)"));
  problem_client->addPredicate(plansys2::Predicate("(connected zone_1_1 room_1)"));
  problem_client->addPredicate(plansys2::Predicate("(connected room_1 zone_1_2)"));
  problem_client->addPredicate(plansys2::Predicate("(connected zone_1_2 room_1)"));
  problem_client->addPredicate(plansys2::Predicate("(connected room_2 zone_recharge)"));
  problem_client->addPredicate(plansys2::Predicate("(connected zone_recharge room_2)"));
  problem_client->addPredicate(plansys2::Predicate("(charging_point_at zone_recharge)"));
  problem_client->addPredicate(plansys2::Predicate("(battery_low leia)"));
  problem_client->addPredicate(plansys2::Predicate("(robot_at leia zone_1_1)"));

  problem_client->setGoal(plansys2::Goal("(and(robot_at leia zone_1_2))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  // The logger must hear every action from its start
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return execution_logger->connected();}));
  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  rclcpp::Rate rate(5);
  while (executor_client->execute_and_check_plan()) {
    rate.sleep();
  }

  auto result = executor_client->getResult();

  ASSERT_EQ(result.value().result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);

  // The actions run in the order of the plan, whichever planner made it
  ASSERT_GE(plan.value().items.size(), 2u);
  ASSERT_TRUE(
    execution_logger->sorted(plansys2_tests::ExecutionLogger::plan_order(plan.value())));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Before the process exits: Zenoh aborts if ROS is shut down by atexit
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
