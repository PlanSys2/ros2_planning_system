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

// The action executors of a node share one actions_hub publisher and subscription

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

namespace
{
size_t endpoints(
  const rclcpp::Node::SharedPtr & observer, const std::string & node_name, bool publishers)
{
  auto infos = publishers ? observer->get_publishers_info_by_topic("/actions_hub") :
    observer->get_subscriptions_info_by_topic("/actions_hub");
  size_t count = 0;
  for (const auto & info : infos) {
    count += info.node_name() == node_name;
  }
  return count;
}
}  // namespace

TEST(action_hub, one_per_node)
{
  auto node_1 = rclcpp_lifecycle::LifecycleNode::make_shared("action_hub_node_1");
  auto node_2 = rclcpp_lifecycle::LifecycleNode::make_shared("action_hub_node_2");

  auto hub = plansys2::ActionHub::get(node_1);
  EXPECT_EQ(plansys2::ActionHub::get(node_1), hub);
  EXPECT_NE(plansys2::ActionHub::get(node_2), hub);

  // Kept while someone holds it, then a new one
  std::weak_ptr<plansys2::ActionHub> weak = hub;
  hub.reset();
  EXPECT_TRUE(weak.expired());
}

TEST(action_hub, the_actions_of_a_node_share_it)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("action_hub_shared");
  auto observer = rclcpp::Node::make_shared("action_hub_observer");

  std::vector<plansys2::ActionExecutor::Ptr> actions;
  for (int i = 0; i < 10; i++) {
    actions.push_back(
      plansys2::ActionExecutor::make_shared("(move r2d2 wp" + std::to_string(i) + " wp9)", node));
  }

  // Ten actions made ten publishers and ten subscriptions
  auto published = [&]() {return endpoints(observer, node->get_name(), true) > 0;};
  ASSERT_TRUE(plansys2::testing::wait_for(published));

  std::this_thread::sleep_for(300ms);  // Time for more to show up, if there were more
  EXPECT_EQ(endpoints(observer, node->get_name(), true), 1u);
  EXPECT_EQ(endpoints(observer, node->get_name(), false), 1u);

  // Gone with the last action
  actions.clear();
  EXPECT_TRUE(
    plansys2::testing::wait_for([&]() {return endpoints(observer, node->get_name(), true) == 0;}));
}

TEST(action_hub, every_action_gets_the_messages)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("action_hub_delivery");
  auto performer = rclcpp::Node::make_shared("action_hub_performer");
  std::vector<plansys2_msgs::msg::ActionExecution> requests;
  auto requests_sub = performer->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable(),
    [&](plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      if (msg->type == plansys2_msgs::msg::ActionExecution::REQUEST) {
        requests.push_back(*msg);
      }
    });
  auto answers = performer->create_publisher<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable());

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(performer);

  auto move = plansys2::ActionExecutor::make_shared("(move r2d2 wp1 wp2)", node);
  auto charge = plansys2::ActionExecutor::make_shared("(charge r2d2)", node);
  {
    plansys2::testing::ExecutorThread spinner(exe);
    ASSERT_TRUE(
      plansys2::testing::wait_for(
        [&]() {
          return answers->get_subscription_count() > 0 && requests_sub->get_publisher_count() > 0;
        }));

    // Each one asks for a performer, and only its own answer starts it
    move->tick(node->now());
    charge->tick(node->now());
    ASSERT_TRUE(plansys2::testing::wait_for([&]() {return requests.size() >= 2;}));
    for (const auto & request : requests) {
      auto response = request;
      response.type = plansys2_msgs::msg::ActionExecution::RESPONSE;
      response.node_id = "action_hub_performer";
      answers->publish(response);
    }
    EXPECT_TRUE(
      plansys2::testing::wait_for(
        [&]() {
          return move->get_internal_status() == plansys2::ActionExecutor::RUNNING &&
                 charge->get_internal_status() == plansys2::ActionExecutor::RUNNING;
        }));

    // A cleaned up action gets no more messages
    charge->clean_up();
    for (const auto & request : requests) {
      auto finish = request;
      finish.type = plansys2_msgs::msg::ActionExecution::FINISH;
      finish.node_id = "action_hub_performer";
      finish.success = true;
      answers->publish(finish);
    }
    EXPECT_TRUE(
      plansys2::testing::wait_for(
        [&]() {return move->get_internal_status() == plansys2::ActionExecutor::SUCCESS;}));
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(charge->get_internal_status(), plansys2::ActionExecutor::RUNNING);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
