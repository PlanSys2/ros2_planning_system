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

#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorNodeContingent.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto parameter_node = std::make_shared<rclcpp::Node>("executor");
  parameter_node->declare_parameter("executor_type", "default_executor");
  std::string executor_name;
  parameter_node->get_parameter("executor_type", executor_name);
  std::shared_ptr<plansys2::ExecutorNodeBase> node;
  if (executor_name == "default_executor") {
    node = std::make_shared<plansys2::ExecutorNode>();
  } else if (executor_name == "contingent_executor") {
    node = std::make_shared<plansys2::ExecutorNodeContingent>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("executor_node"), "Unknown executor type %s", executor_name.c_str());
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
