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

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rcutils/logging.h"
#include "rclcpp/rclcpp.hpp"

#include "plansys2_core/NodeNames.hpp"

namespace
{
std::atomic<int> errors {0};
rcutils_logging_output_handler_t previous_handler = nullptr;

void count_errors(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (severity >= RCUTILS_LOG_SEVERITY_ERROR) {
    errors++;
  }
  previous_handler(location, severity, name, timestamp, format, args);
}

bool has(const std::vector<std::string> & names, const std::string & name)
{
  return std::find(names.begin(), names.end(), "/" + name) != names.end();
}
}  // namespace

TEST(node_names, the_graph_while_ros_runs)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("node_names_a");
  plansys2::NodeNames names(node->get_node_base_interface());

  auto found = names.get(*node->get_node_graph_interface());
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(has(*found, "node_names_a"));

  rclcpp::shutdown();
}

TEST(node_names, nothing_once_ros_shuts_down)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("node_names_b");
  plansys2::NodeNames names(node->get_node_base_interface());
  rclcpp::shutdown();

  EXPECT_FALSE(names.get(*node->get_node_graph_interface()).has_value());
}

TEST(node_names, works_again_after_ros_starts_again)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("node_names_c");
  auto names = std::make_unique<plansys2::NodeNames>(node->get_node_base_interface());
  rclcpp::shutdown();
  names.reset();
  node.reset();

  rclcpp::init(0, nullptr);
  node = rclcpp::Node::make_shared("node_names_d");
  names = std::make_unique<plansys2::NodeNames>(node->get_node_base_interface());
  auto found = names->get(*node->get_node_graph_interface());
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(has(*found, "node_names_d"));
  rclcpp::shutdown();
}

TEST(node_names, destroyed_before_shutdown_leaves_no_callback)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("node_names_e");
  auto context = node->get_node_base_interface()->get_context();
  const auto before = context->get_pre_shutdown_callbacks().size();
  for (int i = 0; i < 100; i++) {
    plansys2::NodeNames names(node->get_node_base_interface());
  }
  EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), before);
  rclcpp::shutdown();
}

TEST(node_names, queries_during_shutdown_log_no_error)
{
  // Many rounds: shutdown lands at different points of a query
  for (int round = 0; round < 100; round++) {
    rclcpp::init(0, nullptr);
    // rclcpp::init sets its own handler
    previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(count_errors);
    auto node = rclcpp::Node::make_shared("node_names_f");
    std::vector<std::unique_ptr<plansys2::NodeNames>> names;
    for (int i = 0; i < 4; i++) {
      names.push_back(std::make_unique<plansys2::NodeNames>(node->get_node_base_interface()));
    }
    errors = 0;
    std::atomic<bool> stop {false};
    std::atomic<int> answers {0};
    std::vector<std::thread> threads;
    for (auto & n : names) {
      threads.emplace_back(
        [&, query = n.get()]() {
          while (!stop) {
            if (query->get(*node->get_node_graph_interface())) {
              answers++;
            }
          }
        });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2 + round % 20));
    rclcpp::shutdown();
    // And so does shutdown
    rcutils_logging_set_output_handler(count_errors);
    stop = true;
    for (auto & t : threads) {
      t.join();
    }
    rcutils_logging_set_output_handler(previous_handler);
    EXPECT_GT(answers, 0) << "round " << round;
    EXPECT_EQ(errors, 0) << "round " << round;
  }
}
