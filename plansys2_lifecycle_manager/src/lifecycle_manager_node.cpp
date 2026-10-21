// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <string>
#include <thread>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto main_node = rclcpp::Node::make_shared("lifecycle_manager");
  const auto startup_timeout = main_node->declare_parameter("startup_timeout", 5);
  // Once up, it watches the components: false to exit after the startup, as before 4.0.0
  const auto supervise = main_node->declare_parameter("supervise", true);
  const auto period = main_node->declare_parameter("supervise_period", 0.5);
  plansys2::Supervisor::Options options;
  options.lost_after = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 * main_node->declare_parameter("lost_after", 3.0)));
  options.grace = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 * main_node->declare_parameter("grace", 10.0)));
  options.transition_timeout = std::chrono::seconds(startup_timeout);

  // One node, the manager's: the four clients work on it
  auto client_node = std::make_shared<plansys2::ClientNode>(*main_node);
  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  for (const auto & name : {"domain_expert", "problem_expert", "planner", "executor"}) {
    manager_nodes[name] = std::make_shared<plansys2::LifecycleServiceClient>(client_node, name);
  }

  // The clients bring their answers while they wait: nothing to spin
  if (!plansys2::startup_function(manager_nodes, std::chrono::seconds(startup_timeout))) {
    RCLCPP_ERROR(
      rclcpp::get_logger("plansys2_lifecycle_manager"),
      "Failed to start plansys2!");
    // Watching, it brings up each component when it is there (a process that died while it
    // started comes back unconfigured)
    if (!supervise) {
      rclcpp::shutdown();
      return -1;
    }
  }

  if (supervise) {
    plansys2::Supervisor supervisor(manager_nodes, options);
    const auto step = std::chrono::milliseconds(static_cast<int64_t>(1000.0 * period));
    auto next = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      supervisor.step();
      next += step;
      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() > next + step) {
        next = std::chrono::steady_clock::now();  // A slow step: no burst to catch up
      }
    }
  }

  rclcpp::shutdown();

  return 0;
}
