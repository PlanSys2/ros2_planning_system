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
#include <thread>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto main_node = rclcpp::Node::make_shared("lifecycle_manager");
  const auto startup_timeout = main_node->declare_parameter("startup_timeout", 5);

  // One node, the manager's: the four clients work on it
  auto client_node = std::make_shared<plansys2::ClientNode>(*main_node);
  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  for (const auto & name : {"domain_expert", "problem_expert", "planner", "executor"}) {
    manager_nodes[name] = std::make_shared<plansys2::LifecycleServiceClient>(client_node, name);
  }

  // Its node brings the answers while the startup waits for them
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(main_node);
  std::thread spinner([&executor]() {executor.spin();});

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(
      plansys2::startup_function, manager_nodes, std::chrono::seconds(startup_timeout)));
  startup_future.wait();
  executor.cancel();
  spinner.join();

  if (!startup_future.get()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("plansys2_lifecycle_manager"),
      "Failed to start plansys2!");
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();

  return 0;
}
