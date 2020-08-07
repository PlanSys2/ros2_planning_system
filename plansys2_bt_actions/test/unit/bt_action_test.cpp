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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "../behavior_tree/OpenGripper.hpp"
#include "../behavior_tree/CloseGripper.hpp"
#include "../behavior_tree/Move.hpp"

#include "plansys2_executor/ActionExecutor.hpp"

#include "test_msgs/action/fibonacci.hpp"
#include "plansys2_msgs/action/execute_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_bt_actions/BTAction.hpp"

#include "gtest/gtest.h"

using namespace std::placeholders;

class MoveServer : public rclcpp::Node
{
  using Fibonacci = test_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

public:
  MoveServer() : Node("move_server") {}

  void start_server()
  {
    move_action_server_ = rclcpp_action::create_server<Fibonacci>(
      shared_from_this(),
      "move",
      std::bind(&MoveServer::handle_goal, this, _1, _2),
      std::bind(&MoveServer::handle_cancel, this, _1),
      std::bind(&MoveServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr move_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    std::thread{std::bind(&MoveServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    
    result->sequence.push_back(4);
    goal_handle->succeed(result);
  }
};

TEST(bt_actions, load_plugins)
{
  auto node = rclcpp::Node::make_shared("load_plugins_node");
  auto move_server_node = std::make_shared<MoveServer>();
  move_server_node->start_server();

  bool finish = false;
  std::thread t([&](){
    while (!finish) rclcpp::spin_some(move_server_node);
  });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("plansys2_close_gripper_bt_node"));
  factory.registerFromPlugin(loader.getOSName("plansys2_open_gripper_bt_node"));
  factory.registerFromPlugin(loader.getOSName("plansys2_move_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_actions");
  std::string xml_file = pkgpath + "/test/behavior_tree/transport.xml";

  std::cerr << "[" << xml_file << "]" << std::endl;

  auto blackboard =  BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(10);

  int counter = 0;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    counter++;
    rate.sleep();
  }

  ASSERT_EQ(counter, 12);
  t.join();
}

class ActionClient : public rclcpp::Node
{
  using ExecuteAction = plansys2_msgs::action::ExecuteAction;
  using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;

public:
  ActionClient() : Node("action_client") {}

  void call_server(const std::string & action_name, const std::vector<std::string> & args)
  {
    auto client_ptr = rclcpp_action::create_client<ExecuteAction>(
      shared_from_this(), action_name);
    
    auto goal_msg = ExecuteAction::Goal();
    goal_msg.action = action_name;
    goal_msg.arguments = args;

    action_future_ = client_ptr->async_send_goal(goal_msg);
  }

  std::shared_future<GoalHandleExecuteAction::SharedPtr> action_future_;
};

TEST(bt_actions, bt_action)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_actions");
  std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

  std::vector<std::string> plugins = {"plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};
  auto bt_action = std::make_shared<plansys2::BTAction>(
    "assemble",
    xml_file,
    plugins);
  
  auto action_client = std::make_shared<ActionClient>();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(bt_action->get_node_base_interface());
  exe.add_node(action_client);

  bool finished = false;
  bool first_time = true;
  while (rclcpp::ok && !finished) {
    exe.spin_some();

    if (first_time) {
      action_client->call_server("assemble", {});
      first_time = false;
    }

    finished = action_client->action_future_.valid();
  }
}

// class ActionExecutorClientTest : public plansys2::ActionExecutorClient
// {
// public:
//   ActionExecutorClientTest()
//   : ActionExecutorClient("assemble", 2.0f),
//     counter_(0), executions_(0)
//   {
//   }
// 
//   void atStart()
//   {
//     executions_++;
//     counter_ = 0;
//   }
// 
//   void actionStep()
//   {
//     RCLCPP_INFO(get_logger(), "Executing assemble");
//     counter_++;
//   }
// 
//   bool isFinished()
//   {
//     return counter_ > 5;
//   }
// 
//   int counter_;
//   int executions_;
// };
// 
// class ClienTestA : public plansys2::ActionBTExecutorClient
// {
// public:
//   ClienTestA()
//   : ActionBTExecutorClient(
//       "assemble",
//       ament_index_cpp::get_package_share_directory("plansys2_executor") +
//       "/assemble.xml",
//       2.0f),
//     executions_(0)
//   {
//     factory_.registerNodeType<plansys2_bt_tests::OpenGripper>("OpenGripper");
//     factory_.registerNodeType<plansys2_bt_tests::CloseGripper>("CloseGripper");
//   }
// 
//   void atStart()
//   {
//     executions_++;
//     ActionBTExecutorClient::atStart();
//   }
// 
//   void actionStep()
//   {
//     ActionBTExecutorClient::actionStep();
//   }
// 
//   bool isFinished()
//   {
//     return ActionBTExecutorClient::isFinished();
//   }
// 
//   int executions_;
// };
// 
// class ClienTestB : public plansys2::ActionBTExecutorClient
// {
// public:
//   ClienTestB()
//   : ActionBTExecutorClient(
//       "transport",
//       ament_index_cpp::get_package_share_directory("plansys2_executor") +
//       "/transport.xml",
//       2.0f),
//     executions_(0),
//     final_value_(0)
//   {
//     factory_.registerNodeType<plansys2_bt_tests::Move>("Move");
//     factory_.registerNodeType<plansys2_bt_tests::OpenGripper>("OpenGripper");
//     factory_.registerNodeType<plansys2_bt_tests::CloseGripper>("CloseGripper");
// 
//     geometry_msgs::msg::Pose2D pose;
//     pose.x = 6;
//     getBackboard()->set("goal", pose);
//   }
// 
//   void atStart()
//   {
//     final_value_ = 0;
//     executions_++;
//     ActionBTExecutorClient::atStart();
//   }
// 
//   void atSuccess()
//   {
//     getBackboard()->get("goal_reached", final_value_);
//   }
// 
//   bool isFinished()
//   {
//     return ActionBTExecutorClient::isFinished();
//   }
// 
//   int executions_;
//   int final_value_;
// };
// 
// TEST(executor_client, normal_client)
// {
//   using ExecuteAction = plansys2_msgs::action::ExecuteAction;
//   using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;
// 
//   auto exc = std::make_shared<ActionExecutorClientTest>();
//   auto test_node = std::make_shared<rclcpp::Node>("test_node");
//   auto action_client = rclcpp_action::create_client<ExecuteAction>(test_node, "assemble");
// 
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(exc->get_node_base_interface());
//   executor.add_node(test_node);
// 
// 
//   RCLCPP_INFO(test_node->get_logger(), "Waiting for action server");
//   action_client->wait_for_action_server();
// 
//   auto goal_msg = ExecuteAction::Goal();
//   goal_msg.action = "assemble";
//   auto send_goal_options = rclcpp_action::Client<ExecuteAction>::SendGoalOptions();
//   auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
// 
//   ASSERT_EQ(exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
// 
//   ASSERT_EQ(exc->executions_, 0);
//   ASSERT_EQ(exc->counter_, 0);
// 
//   rclcpp::Rate rate(10);
//   auto start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 3) {
//     executor.spin_some();
//     rate.sleep();
// 
//     if (!exc->isFinished()) {
//       // ASSERT_EQ(exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
//     }
//   }
// 
//   ASSERT_EQ(exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
//   ASSERT_EQ(exc->executions_, 1);
//   ASSERT_EQ(exc->counter_, 6);
// 
//   future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
// 
//   start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 3) {
//     executor.spin_some();
//     rate.sleep();
// 
//     if (!exc->isFinished()) {
//       ASSERT_EQ(exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
//     }
//   }
// 
//   ASSERT_EQ(exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
//   ASSERT_EQ(exc->executions_, 2);
//   ASSERT_EQ(exc->counter_, 6);
// }
// 
// TEST(executor_client, bt_client)
// {
//   using ExecuteAction = plansys2_msgs::action::ExecuteAction;
//   using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;
// 
//   auto bt_exc = std::make_shared<ClienTestA>();
//   auto test_node = std::make_shared<rclcpp::Node>("test_node");
//   auto action_client = rclcpp_action::create_client<ExecuteAction>(test_node, "assemble");
// 
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(bt_exc->get_node_base_interface());
//   executor.add_node(test_node);
// 
// 
//   RCLCPP_INFO(test_node->get_logger(), "Waiting for action server");
//   action_client->wait_for_action_server();
// 
//   auto goal_msg = ExecuteAction::Goal();
//   goal_msg.action = "assemble";
//   auto send_goal_options = rclcpp_action::Client<ExecuteAction>::SendGoalOptions();
//   auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
// 
//   ASSERT_EQ(bt_exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
// 
//   ASSERT_EQ(bt_exc->executions_, 0);
// 
//   rclcpp::Rate rate(10);
//   while (rclcpp::ok() && !bt_exc->isFinished()) {
//     executor.spin_some();
//     rate.sleep();
//   }
//   auto start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 1) {
//     executor.spin_some();
//     rate.sleep();
//   }
// 
//   // ASSERT_EQ(bt_exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
//   ASSERT_EQ(bt_exc->executions_, 1);
// 
//   future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
// 
//   start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 1) {
//     executor.spin_some();
//     rate.sleep();
//   }
//   while (rclcpp::ok() && !bt_exc->isFinished()) {
//     executor.spin_some();
//     rate.sleep();
//   }
//   start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 1) {
//     executor.spin_some();
//     rate.sleep();
//   }
// 
//   ASSERT_EQ(bt_exc->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
//   ASSERT_EQ(bt_exc->executions_, 2);
// }
// 
// TEST(executor_client, bt_client_b)
// {
//   using ExecuteAction = plansys2_msgs::action::ExecuteAction;
//   using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;
//
//   auto bt_exc = std::make_shared<ClienTestB>();
//   auto test_node = std::make_shared<rclcpp::Node>("test_node");
//   auto action_client = rclcpp_action::create_client<ExecuteAction>(test_node, "transport");
//   auto test_action_server = std::make_shared<RepeaterServer>();
//
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(bt_exc->get_node_base_interface());
//   executor.add_node(test_node);
//   executor.add_node(test_action_server);
//
//   test_action_server->start_server();
//
//   RCLCPP_INFO(test_node->get_logger(), "Waiting for action server");
//   action_client->wait_for_action_server();
//
//   auto goal_msg = ExecuteAction::Goal();
//   goal_msg.action = "transport";
//   auto send_goal_options = rclcpp_action::Client<ExecuteAction>::SendGoalOptions();
//   auto future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
//
//   ASSERT_EQ(bt_exc->executions_, 0);
//   ASSERT_EQ(bt_exc->final_value_, 0);
//
//   rclcpp::Rate rate(10);
//   while (rclcpp::ok() && !bt_exc->isFinished()) {
//     executor.spin_some();
//     rate.sleep();
//   }
//   auto start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 1) {
//     executor.spin_some();
//     rate.sleep();
//   }
//
//   ASSERT_EQ(bt_exc->executions_, 1);
//   // ASSERT_EQ(bt_exc->final_value_, 6);
//
//
//   future_goal_handle = action_client->async_send_goal(goal_msg, send_goal_options);
//
//   start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 1) {
//     executor.spin_some();
//     rate.sleep();
//   }
//
//   // ASSERT_EQ(bt_exc->final_value_, 0);
//
//
//   while (rclcpp::ok() && !bt_exc->isFinished()) {
//     executor.spin_some();
//     rate.sleep();
//   }
//   start = test_node->now();
//   while (rclcpp::ok() && (test_node->now() - start).seconds() < 1) {
//     executor.spin_some();
//     rate.sleep();
//   }
//
//   // ASSERT_EQ(bt_exc->executions_, 2);
//   // ASSERT_EQ(bt_exc->final_value_, 6);
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
