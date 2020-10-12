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


TEST(problem_expert, action_executor_api)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto action_executor_1 = plansys2::ActionExecutor::make_shared("action_1", node);

  ASSERT_EQ(action_executor_1->get_status(), BT::NodeStatus::IDLE);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<MoveAction>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<MoveAction>(node_name, rate);
  }


  MoveAction(const std::string & id, const std::chrono::nanoseconds & rate)
  : ActionExecutorClient(id, rate)
  {
    executions_ = 0;
    cycles_ = 0;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State & state)
  {
    std::cerr << "MoveAction::on_activate" << std::endl;
    counter_ = 0;

    return ActionExecutorClient::on_activate(state);
  }

  void do_work() override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_managed_ << "]");
    for (const auto & arg : current_arguments_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << arg << "]");
    }

    cycles_++;

    if (counter_++ > 3) {
      finish(true, 1.0, "completed");
      executions_++;
    } else {
      send_feedback(counter_ * 0.0, "running");
    }
  }

  int counter_;
  int executions_;
  int cycles_;
};

class TransportAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<MoveAction>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<MoveAction>(node_name, rate);
  }


  TransportAction(const std::string & id, const std::chrono::nanoseconds & rate)
  : ActionExecutorClient(id, rate)
  {
    executions_ = 0;
    cycles_ = 0;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State & state)
  {
    std::cerr << "TransportAction::on_activate" << std::endl;
    counter_ = 0;

    return ActionExecutorClient::on_activate(state);
  }

  void do_work() override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_managed_ << "]");
    for (const auto & arg : current_arguments_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << arg << "]");
    }

    cycles_++;

    if (counter_++ > 3) {
      finish(true, 1.0, "completed");
      executions_++;
    } else {
      send_feedback(counter_ * 0.0, "running");
    }
  }

  int counter_;
  int executions_;
  int cycles_;
};


TEST(problem_expert, action_executor_client)
{
  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto aux_node = rclcpp_lifecycle::LifecycleNode::make_shared("aux_node");

  auto move_action_1_node = MoveAction::make_shared("move_node_1", 1s);
  auto move_action_2_node = MoveAction::make_shared("move_node_2", 1s);
  auto transport_action_node = TransportAction::make_shared("transport_node", 500ms);

  move_action_1_node->set_parameter({"action", "move"});
  move_action_2_node->set_parameter({"action", "move"});
  transport_action_node->set_parameter({"action", "transport"});

  move_action_1_node->set_parameter(
    {"specialized_arguments", std::vector<std::string>({"robot1"})});
  move_action_2_node->set_parameter(
    {"specialized_arguments", std::vector<std::string>({"robot2"})});

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  aux_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_1_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_2_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  transport_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  aux_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(test_node->get_node_base_interface());
  exe.add_node(aux_node->get_node_base_interface());
  exe.add_node(move_action_1_node->get_node_base_interface());
  exe.add_node(move_action_2_node->get_node_base_interface());
  exe.add_node(transport_action_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  std::vector<plansys2_msgs::msg::ActionExecution> history_msgs;
  bool confirmed = false;
  auto actions_sub = aux_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub",
    rclcpp::QoS(100).reliable(), [&](plansys2_msgs::msg::ActionExecution::UniquePtr msg) {
      history_msgs.push_back(*msg);
    });

  std::string bt_xml_tree =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <Parallel threshold="2">
            <ExecuteAction    action="(move robot1 wheels_zone assembly_zone):5"/>
            <ExecuteAction    action="(move robot1 steering_wheels_zone assembly_zone):5"/>
          </Parallel>
          <ExecuteAction    action="(transport robot2 steering_wheels_zone assembly_zone):10"/>
          <ExecuteAction    action="(move robot2 wheels_zone assembly_zone):15"/>
          <ExecuteAction    action="(transport robot2 steering_wheels_zone assembly_zone):20"/>
          <ExecuteAction    action="(move robot1 wheels_zone assembly_zone):25"/>
          <ExecuteAction    action="(transport robot2 steering_wheels_zone assembly_zone):30"/>
          <ExecuteAction    action="(move robot1 steering_wheels_zone assembly_zone):35"/>
       </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto blackboard = BT::Blackboard::create();

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  blackboard->set("action_map", action_map);

  blackboard->set("node", test_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<plansys2::ExecuteAction>("ExecuteAction");
  factory.registerNodeType<plansys2::WaitAction>("WaitAction");

  auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);


  auto status = BT::NodeStatus::RUNNING;
  while (status != BT::NodeStatus::SUCCESS) {
    status = tree.tickRoot();
  }

  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(move_action_1_node->executions_, 4);
  ASSERT_EQ(move_action_1_node->cycles_, 20);
  ASSERT_EQ(move_action_2_node->executions_, 1);
  ASSERT_EQ(move_action_2_node->cycles_, 5);
  ASSERT_EQ(transport_action_node->executions_, 3);
  ASSERT_EQ(transport_action_node->cycles_, 15);
  // ASSERT_EQ(history_msgs.size(), 56);


  finish = true;
  t.join();
}

TEST(problem_expert, action_executor)
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

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and(car_assembled car_1))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::map<std::string, plansys2::DurativeAction> durative_actions_map;
  plansys2::BTBuilder exec_tree(test_node);
  auto tree_str = exec_tree.get_tree(plan.value());

  finish = true;
  t.join();
}

TEST(problem_expert, executor_client)
{
  auto test_node = rclcpp::Node::make_shared("executor_client_test");
  auto test_node2 = rclcpp::Node::make_shared("executor_client_test2");

  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(test_node);
  auto executor_client = std::make_shared<plansys2::ExecutorClient>(test_node2);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

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

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and(car_assembled car_1))")));
  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());

  ASSERT_TRUE(plan);

  // bool success = executor_client->executePlan();
  // ASSERT_TRUE(success);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
