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

#include <memory>

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_core/ClientNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plansys2::DomainExpertNode>();

  // Multi-threaded, as in plansys2_node: a callback may wait for a client of the node.
  // EventsExecutor is deprecated; EventsCBGExecutor delays bursts of actions_hub messages (#449)
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), plansys2::kComponentThreads);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
