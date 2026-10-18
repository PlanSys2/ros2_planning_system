// Copyright 2026 Intelligent Robotics Lab
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

// BtActionNode and BtServiceNode, tick by tick, against servers that answer as each test says

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_bt_actions/BTAction.hpp"
#include "plansys2_bt_actions/BTActionNode.hpp"
#include "plansys2_bt_actions/BTServiceNode.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "test_msgs/action/fibonacci.hpp"
#include "test_msgs/srv/empty.hpp"

using namespace std::chrono_literals;  // NOLINT
using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

namespace
{

// An action server whose answers the test sets
class TestServer : public rclcpp::Node
{
public:
  TestServer()
  : Node("bt_action_node_test_server")
  {
  }

  void start()
  {
    server_ = rclcpp_action::create_server<Fibonacci>(
      shared_from_this(), "test_action",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        goals++;
        std::this_thread::sleep_for(accept_delay);
        return reject ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        auto duration = run_for;
        std::thread{[goal_handle, duration]() {
            auto end = std::chrono::steady_clock::now() + duration;
            while (std::chrono::steady_clock::now() < end) {
              if (goal_handle->is_canceling()) {
                goal_handle->canceled(std::make_shared<Fibonacci::Result>());
                return;
              }
              std::this_thread::sleep_for(10ms);
            }
            if (goal_handle->is_active()) {
              goal_handle->succeed(std::make_shared<Fibonacci::Result>());
            }
          }}.detach();
      });
  }

  std::atomic<int> goals {0};
  std::chrono::milliseconds accept_delay {0};
  std::chrono::milliseconds run_for {0};
  std::atomic<bool> reject {false};

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
};

// The plainest action node
class Plain : public plansys2::BtActionNode<Fibonacci>
{
public:
  Plain(const std::string & name, const BT::NodeConfig & conf)
  : BtActionNode(name, "test_action", conf) {}

  static BT::PortsList providedPorts() {return providedBasicPorts({});}

  // On its second tick it asks to send the goal again, once
  BT::NodeStatus on_tick() override
  {
    if (update_on_second_tick && ++ticks == 2) {
      goal_updated_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }

  static inline bool update_on_second_tick {false};
  int ticks {0};
};

class PlainService : public plansys2::BtServiceNode<test_msgs::srv::Empty>
{
public:
  PlainService(const std::string & name, const BT::NodeConfig & conf)
  : BtServiceNode(name, conf, "test_service") {}

  static BT::PortsList providedPorts() {return providedBasicPorts({});}
};

}  // namespace

class BtActionNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("bt_action_node_test");
    server_ = std::make_shared<TestServer>();
    server_->start();
    exe_.add_node(server_);
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", node_);
    blackboard_->set<std::chrono::milliseconds>("server_timeout", 1000ms);
    blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);

    factory_.registerNodeType<Plain>("Plain");
    factory_.registerNodeType<PlainService>("PlainService");
    Plain::update_on_second_tick = false;
  }

  // Ticks the tree every 100 ms until it ends, or the time is up
  BT::NodeStatus run(BT::Tree & tree, std::chrono::milliseconds timeout = 5000ms)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    auto status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
      std::this_thread::sleep_for(100ms);
    }
    return status;
  }

  BT::Tree tree(const std::string & body)
  {
    return factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" + body +
      "</BehaviorTree></root>", blackboard_);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<TestServer> server_;
  rclcpp::executors::SingleThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
};

// A server that takes longer than a tick to accept the goal has not rejected it
TEST_F(BtActionNodeTest, slow_acceptance_is_not_a_rejection)
{
  server_->accept_delay = 400ms;
  auto bt = tree("<Plain/>");
  ASSERT_EQ(run(bt), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(server_->goals, 1);
}

// A rejected goal fails the node, without an exception out of the tick
TEST_F(BtActionNodeTest, rejected_goal_fails_the_node)
{
  server_->reject = true;
  auto bt = tree("<Plain/>");
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  ASSERT_NO_THROW(status = run(bt));
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

// Repeat runs the action every time: the node is ready again after each goal
TEST_F(BtActionNodeTest, repeat_runs_the_action_each_time)
{
  server_->run_for = 200ms;
  auto bt = tree("<Repeat num_cycles=\"3\"><Plain/></Repeat>");
  ASSERT_EQ(run(bt, 10000ms), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(server_->goals, 3);
}

// After a halt, the next tick sends a new goal instead of going on with the old one
TEST_F(BtActionNodeTest, halt_then_tick_sends_a_new_goal)
{
  server_->run_for = 3000ms;
  auto bt = tree("<Plain/>");
  for (int i = 0; i < 4; i++) {
    ASSERT_EQ(bt.tickOnce(), BT::NodeStatus::RUNNING);
    // Kept: ticks spaced as in run(), while the goal runs
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_EQ(server_->goals, 1);
  bt.haltTree();

  server_->run_for = 200ms;
  ASSERT_EQ(run(bt), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(server_->goals, 2);
}

// Without server_timeout in the XML, the one of the blackboard counts
TEST_F(BtActionNodeTest, server_timeout_comes_from_the_blackboard)
{
  blackboard_->set<std::chrono::milliseconds>("server_timeout", 300ms);
  server_->accept_delay = 2000ms;
  auto bt = tree("<Plain/>");
  auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(run(bt), BT::NodeStatus::FAILURE);
  // After the 300 ms of the blackboard: not at once, nor after the 5 s of a default
  auto elapsed = std::chrono::steady_clock::now() - start;
  ASSERT_GE(elapsed, 250ms);
  ASSERT_LT(elapsed, 1500ms);
}

// An updated goal is sent once, not on every tick
TEST_F(BtActionNodeTest, updated_goal_is_sent_once)
{
  Plain::update_on_second_tick = true;
  server_->run_for = 1000ms;
  auto bt = tree("<Plain/>");
  ASSERT_EQ(run(bt), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(server_->goals, 2);
}

// BtServiceNode keeps the service name given to its constructor when the XML gives none
TEST_F(BtActionNodeTest, service_node_keeps_its_service_name)
{
  auto service_node = rclcpp::Node::make_shared("bt_action_node_test_service");
  std::atomic<int> calls {0};
  auto service = service_node->create_service<test_msgs::srv::Empty>(
    "test_service",
    [&calls](const std::shared_ptr<test_msgs::srv::Empty::Request>,
    std::shared_ptr<test_msgs::srv::Empty::Response>) {calls++;});
  // The node waits for its service
  exe_.add_node(service_node);

  auto bt = tree("<PlainService/>");
  ASSERT_EQ(run(bt), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(calls, 1);
  exe_.remove_node(service_node);
}

// Without the service, the error names it
TEST_F(BtActionNodeTest, service_node_without_server_names_it)
{
  blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", 200ms);
  try {
    auto bt = tree("<PlainService/>");
    FAIL() << "No exception";
  } catch (const std::exception & e) {
    EXPECT_NE(std::string(e.what()).find("test_service"), std::string::npos) << e.what();
  }
}

// Without node in the blackboard the node cannot be built, and says why
TEST_F(BtActionNodeTest, no_node_in_the_blackboard)
{
  blackboard_ = BT::Blackboard::create();
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
  try {
    auto bt = tree("<Plain/>");
    FAIL() << "No exception";
  } catch (const std::exception & e) {
    EXPECT_NE(std::string(e.what()).find("node"), std::string::npos) << e.what();
  }
}

// BTActions created with the same parent blackboard share its '@' keys (#315)
TEST(BTActionBlackboard, actions_share_the_parent_blackboard)
{
  auto parent = BT::Blackboard::create();
  auto first = std::make_shared<plansys2::BTAction>("first_bt_action", parent);
  auto second = std::make_shared<plansys2::BTAction>("second_bt_action", parent);
  auto alone = std::make_shared<plansys2::BTAction>("alone_bt_action");
  for (auto & action : {first, second, alone}) {
    action->set_parameter({"action_name", "move"});
    action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_NE(action->getBlackboard(), nullptr);
  }

  first->getBlackboard()->set<int>("@battery", 42);
  ASSERT_EQ(second->getBlackboard()->get<int>("@battery"), 42);
  ASSERT_EQ(parent->get<int>("battery"), 42);

  // Each one keeps its own keys, and one without parent shares nothing
  first->getBlackboard()->set<int>("local", 1);
  int value = 0;
  EXPECT_FALSE(second->getBlackboard()->get<int>("local", value));
  EXPECT_FALSE(alone->getBlackboard()->get<int>("@battery", value));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
