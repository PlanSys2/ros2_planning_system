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

// The helpers of the tests: a spinning thread that always ends, and waiting for conditions

#include <atomic>
#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "gtest/gtest-spi.h"
#include "plansys2_core/TestUtils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

TEST(test_utils, executor_thread_spins_until_destroyed)
{
  auto node = rclcpp::Node::make_shared("test_utils_spin");
  std::atomic<int> received {0};
  auto sub = node->create_subscription<std_msgs::msg::Empty>(
    "test_utils_topic", 10, [&](std_msgs::msg::Empty::SharedPtr) {received++;});
  auto pub = node->create_publisher<std_msgs::msg::Empty>("test_utils_topic", 10);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  {
    plansys2::testing::ExecutorThread spinner(exe);
    ASSERT_TRUE(
      plansys2::testing::wait_for(
        [&]() {
          pub->publish(std_msgs::msg::Empty());
          return received > 0;
        }, 5s));
  }
  // Destroyed: nothing spins the node any more
  const int before = received;
  pub->publish(std_msgs::msg::Empty());
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(received, before);
}

// A failed ASSERT returns while the thread runs: it must end, not abort the program
rclcpp::Executor * failing_executor = nullptr;

void fails_while_spinning()
{
  plansys2::testing::ExecutorThread spinner(*failing_executor);
  ASSERT_TRUE(false) << "expected failure";
}

TEST(test_utils, executor_thread_ends_on_a_failed_assert)
{
  auto node = rclcpp::Node::make_shared("test_utils_assert");
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  failing_executor = &exe;
  EXPECT_FATAL_FAILURE(fails_while_spinning(), "expected failure");
  failing_executor = nullptr;
}

TEST(test_utils, wait_for)
{
  EXPECT_TRUE(plansys2::testing::wait_for([]() {return true;}, 0s));
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(plansys2::testing::wait_for([]() {return false;}, 200ms));
  const auto waited = std::chrono::steady_clock::now() - start;
  EXPECT_GE(waited, 200ms);
  EXPECT_LT(waited, 2s);

  int calls = 0;
  EXPECT_TRUE(plansys2::testing::wait_for([&]() {return ++calls == 5;}, 5s));
  EXPECT_EQ(calls, 5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
