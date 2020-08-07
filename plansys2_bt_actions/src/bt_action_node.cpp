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

#include "plansys2_bt_actions/BTAction.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("default");

  if (std::string(node->get_name()) == "default")
  {
    RCLCPP_ERROR(node->get_logger(), "Node name must be set externally");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Node name: [" << node->get_name() << "]");
  
  node->declare_parameter("action_name");
  node->declare_parameter("bt_xml_file");
  node->declare_parameter("plugins");

  std::string action_name;
  node->get_parameter("action_name", action_name);
  std::string bt_xml_file;
  node->get_parameter("bt_xml_file", bt_xml_file);

  RCLCPP_INFO_STREAM(node->get_logger(), "action_name: [" << action_name << "]");
  RCLCPP_INFO_STREAM(node->get_logger(), "bt_xml_file: [" << bt_xml_file << "]");

  auto plugin_lib_names_ = node->get_parameter("plugins").as_string_array();
  for (auto plugin : plugin_lib_names_)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "plugin: [" << plugin << "]");
  }

  auto action_node = std::make_shared<plansys2::BTAction>(
    action_name,
    bt_xml_file,
    plugin_lib_names_
  );

  rclcpp::spin(action_node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
