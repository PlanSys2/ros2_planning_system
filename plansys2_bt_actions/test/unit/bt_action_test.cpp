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

#include <chrono>
#include <string>
#include <vector>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <iostream>
#include <fstream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "../behavior_tree/OpenGripper.hpp"
#include "../behavior_tree/CloseGripper.hpp"
#include "../behavior_tree/Move.hpp"
#include "../behavior_tree/FailureNodes.hpp"

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"

#include "test_msgs/action/fibonacci.hpp"
#include "plansys2_core/PackagePaths.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_bt_actions/BTAction.hpp"

#include "gtest/gtest.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MoveServer : public rclcpp::Node
{
  using Fibonacci = test_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

public:
  MoveServer()
  : Node("move_server") {}

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
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Fibonacci::Goal>)
  {
    RCLCPP_DEBUG(get_logger(), "handle_goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci>)
  {
    cancelled_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handle_accepted");
    // The thread keeps the server alive: it may outlast the test that created it
    auto self = std::static_pointer_cast<MoveServer>(shared_from_this());
    std::thread{[self, goal_handle]() {self->execute(goal_handle);}}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    auto feedback = std::make_shared<Fibonacci::Feedback>();

    int counter = 0;
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && counter++ < 50 && !cancelled_) {  // 5 secs
      RCLCPP_DEBUG(get_logger(), "execute");
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    if (!cancelled_) {
      auto result = std::make_shared<Fibonacci::Result>();

      result->sequence.push_back(4);
      goal_handle->succeed(result);
    }
  }

  std::atomic<bool> cancelled_ {false};
};


// Accepts each goal and succeeds at once
class QuickServer : public rclcpp::Node
{
  using Fibonacci = test_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

public:
  QuickServer()
  : Node("quick_server") {}

  void start_server()
  {
    server_ = rclcpp_action::create_server<Fibonacci>(
      shared_from_this(), "quick",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandleFibonacci>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        std::thread{[goal_handle]() {
            goal_handle->succeed(std::make_shared<Fibonacci::Result>());
          }}.detach();
      });
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
};

class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

class BTActionsTestCase : public ::testing::Test
{
protected:
  void SetUp()
  {
  }

  void TearDown()
  {
  }
};


TEST_F(BTActionsTestCase, load_plugins)
{
  {
    auto node = rclcpp_lifecycle::LifecycleNode::make_shared("load_plugins_node");
    auto move_server_node = std::make_shared<MoveServer>();
    move_server_node->start_server();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(move_server_node);
    exe.add_node(node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("plansys2_close_gripper_bt_node"));
    factory.registerFromPlugin(loader.getOSName("plansys2_open_gripper_bt_node"));
    factory.registerFromPlugin(loader.getOSName("plansys2_move_bt_test_node"));

    std::string pkgpath = plansys2::share_path("plansys2_bt_actions").string();
    std::string xml_file = pkgpath + "/test/behavior_tree/transport.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set<std::chrono::milliseconds>("server_timeout", 250ms);
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

    rclcpp::Rate rate(10);

    bool finish = false;
    while (!finish && rclcpp::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      rate.sleep();
    }

    node->shutdown();
}
  plansys2::drain_ros(200ms);
}

TEST_F(BTActionsTestCase, on_tick_failure)
{
  {
    auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    auto move_server_node = std::make_shared<MoveServer>();
    move_server_node->start_server();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(move_server_node);
    exe.add_node(node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);


    BT::NodeConfig config;
    BT::assignDefaultRemapping<plansys2_bt_tests::OnTickFail>(config);
    auto bb = BT::Blackboard::create();
    bb->set("node", node);
    bb->set<std::chrono::milliseconds>("server_timeout", 250ms);
    bb->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    bb->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
    config.blackboard = bb;

    plansys2_bt_tests::OnTickFail failure_node("OnTickFail", "move", config);

    rclcpp::Rate rate(10);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      status = failure_node.executeTick();
      rate.sleep();
    }

    ASSERT_TRUE(failure_node.on_tick_run);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);

    node->shutdown();
}
  plansys2::drain_ros(200ms);
}

TEST_F(BTActionsTestCase, on_feedback_failure)
{
  {
    auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    auto move_server_node = std::make_shared<MoveServer>();
    move_server_node->start_server();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(move_server_node);
    exe.add_node(node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    BT::NodeConfig config;
    BT::assignDefaultRemapping<plansys2_bt_tests::OnFeedbackFail>(config);
    auto bb = BT::Blackboard::create();
    bb->set("node", node);
    bb->set<std::chrono::milliseconds>("server_timeout", 250ms);
    bb->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    bb->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
    config.blackboard = bb;

    plansys2_bt_tests::OnFeedbackFail failure_node("OnFeedbackFail",
      "move",
      config);

    rclcpp::Rate rate(10);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      status = failure_node.executeTick();
      rate.sleep();
    }

    ASSERT_TRUE(failure_node.on_feedback_run);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);

    node->shutdown();
}
  plansys2::drain_ros(200ms);
}

// The node fails while its goal has just finished on the server: cancelling a goal that
// rclcpp_action already forgot threw from tick() (UnknownGoalHandleError)
TEST_F(BTActionsTestCase, fail_after_the_goal_finished)
{
  {
    auto node = rclcpp_lifecycle::LifecycleNode::make_shared("finished_goal_node");
    auto server_node = std::make_shared<QuickServer>();
    server_node->start_server();
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(server_node);
    exe.add_node(node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    BT::NodeConfig config;
    BT::assignDefaultRemapping<plansys2_bt_tests::FailOnSecondTick>(config);
    auto bb = BT::Blackboard::create();
    bb->set("node", node);
    bb->set<std::chrono::milliseconds>("server_timeout", 1000ms);
    bb->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    bb->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
    config.blackboard = bb;

    plansys2_bt_tests::FailOnSecondTick bt_node("FailOnSecondTick", "quick", config);

    // Sends the goal, then waits until it is accepted
    ASSERT_EQ(bt_node.executeTick(), BT::NodeStatus::RUNNING);
    ASSERT_EQ(bt_node.executeTick(), BT::NodeStatus::RUNNING);
    // The goal succeeds and its result reaches the client before the next tick
    // Kept: the result waits in the client's own executor, which only a tick spins
    std::this_thread::sleep_for(500ms);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    ASSERT_NO_THROW(status = bt_node.executeTick());
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);

    node->shutdown();
  }
  plansys2::drain_ros(200ms);
}

TEST_F(BTActionsTestCase, bt_action)
{
  {
    std::string pkgpath = plansys2::share_path("plansys2_bt_actions").string();
    std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

    std::vector<std::string> plugins = {
      "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

    auto bt_action = std::make_shared<plansys2::BTAction>("assemble");

    auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    auto action_client = plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)",
      lc_node);

    bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
    bt_action->set_parameter(rclcpp::Parameter("rate", 10.0));
    bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
    bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));

    bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(bt_action->get_node_base_interface());
    exe.add_node(lc_node->get_node_base_interface());

    bool finished = false;
    while (rclcpp::ok() && !finished) {
      exe.spin_some();

      action_client->tick(lc_node->now());
      finished = action_client->get_status() == BT::NodeStatus::SUCCESS;
    }

    // The performer deactivates when it finishes
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (bt_action->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
      std::chrono::steady_clock::now() < end)
    {
      exe.spin_some(10ms);
    }

    lc_node->shutdown();
}
  plansys2::drain_ros(200ms);
}

TEST_F(BTActionsTestCase, bt_action_old_constructor)
{
  {
    std::string pkgpath = plansys2::share_path("plansys2_bt_actions").string();
    std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

    std::vector<std::string> plugins = {
      "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

    auto bt_action = std::make_shared<plansys2::BTAction>("assemble");

    auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    auto action_client = plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)",
      lc_node);

    bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
    bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
    bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));

    bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(bt_action->get_node_base_interface());
    exe.add_node(lc_node->get_node_base_interface());

    bool finished = false;
    while (rclcpp::ok() && !finished) {
      exe.spin_some();

      action_client->tick(lc_node->now());
      finished = action_client->get_status() == BT::NodeStatus::SUCCESS;
    }

    // The performer deactivates when it finishes
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (bt_action->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
      std::chrono::steady_clock::now() < end)
    {
      exe.spin_some(10ms);
    }

    lc_node->shutdown();
}
  plansys2::drain_ros(200ms);
}

TEST_F(BTActionsTestCase, cancel_bt_action)
{
  {
    std::string pkgpath = plansys2::share_path("plansys2_bt_actions").string();
    std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

    std::vector<std::string> plugins = {
      "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

    auto bt_action = std::make_shared<plansys2::BTAction>("assemble");

    auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    auto action_client = plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)",
      lc_node);

    bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
    bt_action->set_parameter(rclcpp::Parameter("rate", 1.0));
    bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
    bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));

    bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(bt_action->get_node_base_interface());
    exe.add_node(lc_node->get_node_base_interface());

    // Filled by the spinning thread, read by the test
    std::mutex msgs_mutex;
    std::vector<plansys2_msgs::msg::ActionExecution> received_msgs;
    auto messages = [&]() {
        std::lock_guard<std::mutex> lock(msgs_mutex);
        return received_msgs;
      };

    auto action_hub_sub = lc_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub", rclcpp::QoS(100).reliable(),
      [&](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msgs_mutex);
        received_msgs.push_back(*msg);
    });

    plansys2::testing::ExecutorThread spinner(exe);

    // The test hears the hub of the client and the one of the performer
    ASSERT_TRUE(
      plansys2::testing::wait_for(
        [&]() {return action_hub_sub->get_publisher_count() >= 2;}));

    ASSERT_EQ(action_client->get_internal_status(), plansys2::ActionExecutor::Status::IDLE);
    ASSERT_EQ(
    bt_action->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    // Request, response and confirm
    plansys2::testing::wait_for(
      [&]() {
        action_client->tick(lc_node->now());
        return messages().size() >= 3u &&
               action_client->get_internal_status() == plansys2::ActionExecutor::Status::RUNNING;
      });

    ASSERT_EQ(action_client->get_internal_status(), plansys2::ActionExecutor::Status::RUNNING);
    ASSERT_EQ(bt_action->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto action_execution_msgs = messages();
    ASSERT_EQ(action_execution_msgs.size(), 3u);
    ASSERT_EQ(action_execution_msgs[0].type, plansys2_msgs::msg::ActionExecution::REQUEST);
    ASSERT_EQ(action_execution_msgs[1].type, plansys2_msgs::msg::ActionExecution::RESPONSE);
    ASSERT_EQ(action_execution_msgs[2].type, plansys2_msgs::msg::ActionExecution::CONFIRM);
    // The first feedback, at 1 Hz
    plansys2::testing::wait_for(
      [&]() {
        action_client->tick(lc_node->now());
        return messages().size() >= 4u;
      });

    ASSERT_EQ(messages().size(), 4u);
    action_client->cancel();

    plansys2::testing::wait_for(
      [&]() {
        action_client->tick(lc_node->now());
        return messages().size() >= 5u &&
               action_client->get_internal_status() ==
               plansys2::ActionExecutor::Status::CANCELLED &&
               bt_action->get_current_state().id() ==
               lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      });
    // Kept: time for an unexpected extra message to show up
    std::this_thread::sleep_for(300ms);

    action_execution_msgs = messages();
    ASSERT_EQ(action_execution_msgs.size(), 5u);
    ASSERT_EQ(action_execution_msgs[3].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
    ASSERT_EQ(action_execution_msgs[4].type, plansys2_msgs::msg::ActionExecution::CANCEL);

    ASSERT_EQ(action_client->get_internal_status(), plansys2::ActionExecutor::Status::CANCELLED);
    ASSERT_EQ(
    bt_action->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    lc_node->shutdown();
  }
  plansys2::drain_ros(200ms);
}

// The blackboard keeps a pointer to the node: it must not keep the node alive (#427)
TEST_F(BTActionsTestCase, bt_action_node_is_destroyed)
{
  std::string pkgpath = plansys2::share_path("plansys2_bt_actions").string();
  std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";
  std::vector<std::string> plugins = {
    "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

  auto make_action = [&]() {
      auto bt_action = std::make_shared<plansys2::BTAction>("assemble");
      bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
      bt_action->set_parameter(rclcpp::Parameter("rate", 10.0));
      bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
      bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));
      return bt_action;
    };

  // Never configured
  {
    auto bt_action = make_action();
    std::weak_ptr<plansys2::BTAction> weak = bt_action;
    bt_action.reset();
    ASSERT_TRUE(weak.expired());
  }

  // Configured, with its blackboard filled
  {
    auto bt_action = make_action();
    std::weak_ptr<plansys2::BTAction> weak = bt_action;
    bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(
      bt_action->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    bt_action.reset();
    ASSERT_TRUE(weak.expired());
  }

  // After executing an action, so the tree and its BT nodes were created
  {
    std::weak_ptr<plansys2::BTAction> weak;
    {
      auto bt_action = make_action();
      weak = bt_action;
      auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node_destroyed");
      auto action_client =
        plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)", lc_node);
      bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

      rclcpp::executors::SingleThreadedExecutor exe;
      exe.add_node(bt_action->get_node_base_interface());
      exe.add_node(lc_node->get_node_base_interface());

      auto start = std::chrono::steady_clock::now();
      while (action_client->get_status() != BT::NodeStatus::SUCCESS &&
        std::chrono::steady_clock::now() - start < 30s)
      {
        exe.spin_some();
        action_client->tick(lc_node->now());
      }
      ASSERT_EQ(action_client->get_status(), BT::NodeStatus::SUCCESS);
      lc_node->shutdown();
    }
    ASSERT_TRUE(weak.expired());
  }
  plansys2::drain_ros(200ms);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new ROS2Environment);
  return RUN_ALL_TESTS();
}
