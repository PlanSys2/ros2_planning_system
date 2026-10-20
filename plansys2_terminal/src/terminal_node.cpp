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


#include <csignal>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_terminal/Terminal.hpp"

int main(int argc, char ** argv)
{
  // Ctrl-C is ours: it cancels the plan in execution, or discards the line at the prompt,
  // instead of shutting down ROS and leaving the terminal unable to do anything
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) {plansys2_terminal::interrupt();});
  auto terminal_node = std::make_shared<plansys2_terminal::Terminal>();

  // The console calls the clients; this thread brings their answers
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(terminal_node);
  std::thread spinner([&executor]() {executor.spin();});

  terminal_node->run_console();

  executor.cancel();
  spinner.join();

  rclcpp::shutdown();
  return 0;
}
