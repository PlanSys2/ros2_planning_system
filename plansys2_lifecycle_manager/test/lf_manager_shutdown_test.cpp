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

// Its own binary: it shuts ROS down

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

#include "fake_lifecycle_node.hpp"

using namespace std::chrono_literals;

TEST(lifecycle_manager, shutdown_during_startup)
{
  // Ctrl-C right after the planner accepts its configuration, while startup waits for it
  Spinner spinner;
  auto manager = rclcpp::Node::make_shared("sd_lifecycle_manager");
  plansys2::testing::NodeSpinner manager_spinner(manager);
  auto client_node = std::make_shared<plansys2::ClientNode>(*manager);
  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> clients;
  for (const std::string key : {"planner", "domain_expert", "problem_expert", "executor"}) {
    clients[key] = std::make_shared<plansys2::LifecycleServiceClient>(client_node, "sd_" + key);
  }
  auto fake_planner = std::make_shared<FakeLifecycleNode>("sd_planner");
  fake_planner->on_change_state = []() {
      std::thread([]() {
          std::this_thread::sleep_for(300ms);
          rclcpp::shutdown();
        }).detach();
    };
  spinner.add(fake_planner);
  spinner.start();

  // Long timeout: it must stop because of the shutdown, not because of the timeout
  auto start = std::chrono::steady_clock::now();
  ASSERT_FALSE(plansys2::startup_function(clients, 30s));
  ASSERT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), 5.0);
  ASSERT_GT(fake_planner->get_state_calls.load(), 0);
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
