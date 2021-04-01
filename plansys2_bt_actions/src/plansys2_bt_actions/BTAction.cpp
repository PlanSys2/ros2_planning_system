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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "plansys2_bt_actions/BTAction.hpp"

namespace plansys2
{

BTAction::BTAction(
  const std::string & action,
  const std::chrono::nanoseconds & rate)
: ActionExecutorClient(action, rate)
{
  declare_parameter("bt_xml_file");
  declare_parameter("plugins");
#ifdef ZMQ_FOUND
  declare_parameter<bool>("enable_groot_monitoring", true);
  declare_parameter<int>("publisher_port", -1);
  declare_parameter<int>("server_port", -1);
  declare_parameter<int>("max_msgs_per_second", 25);
#endif
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  get_parameter("action_name", action_);
  get_parameter("bt_xml_file", bt_xml_file_);

  RCLCPP_INFO_STREAM(get_logger(), "action_name: [" << action_ << "]");
  RCLCPP_INFO_STREAM(get_logger(), "bt_xml_file: [" << bt_xml_file_ << "]");

  auto plugin_lib_names = get_parameter("plugins").as_string_array();
  for (auto plugin : plugin_lib_names) {
    RCLCPP_INFO_STREAM(get_logger(), "plugin: [" << plugin << "]");
  }

  BT::SharedLibrary loader;

  for (auto plugin : plugin_lib_names) {
    factory_.registerFromPlugin(loader.getOSName(plugin));
  }

  auto node = rclcpp::Node::make_shared(std::string(get_name()) + "_aux");
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", node);

  return ActionExecutorClient::on_configure(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  publisher_zmq_.reset();
  return ActionExecutorClient::on_cleanup(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  tree_ = factory_.createTreeFromFile(bt_xml_file_, blackboard_);

  for (int i = 0; i < get_arguments().size(); i++) {
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, get_arguments()[i]);
  }

#ifdef ZMQ_FOUND
  int publisher_port = get_parameter("publisher_port").as_int();
  int server_port = get_parameter("server_port").as_int();
  unsigned int max_msgs_per_second = get_parameter("max_msgs_per_second").as_int();

  if (publisher_port <= 0 || server_port <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "[%s] Groot monitoring ports not provided, disabling Groot monitoring."
      " publisher port: %d, server port: %d",
      get_name(), publisher_port, server_port);
  } else if (get_parameter("enable_groot_monitoring").as_bool()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[%s] Groot monitoring: Publisher port: %d, Server port: %d, Max msgs per second: %d",
      get_name(), publisher_port, server_port, max_msgs_per_second);
    try {
      publisher_zmq_.reset(
        new BT::PublisherZMQ(
          tree_, max_msgs_per_second, publisher_port,
          server_port));
    } catch (const BT::LogicError & exc) {
      RCLCPP_ERROR(get_logger(), "ZMQ error: %s", exc.what());
    }
  }
#endif

  finished_ = false;
  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  publisher_zmq_.reset();
  tree_.haltTree();

  return ActionExecutorClient::on_deactivate(previous_state);
}

void
BTAction::do_work()
{
  if (!finished_) {
    auto result = tree_.rootNode()->executeTick();

    switch (result) {
      case BT::NodeStatus::SUCCESS:
        finish(true, 1.0, "Action completed");
        finished_ = true;
        break;
      case BT::NodeStatus::RUNNING:
        send_feedback(0.0, "Action running");
        break;
      case BT::NodeStatus::FAILURE:
        finish(false, 1.0, "Action failed");
        finished_ = true;
        break;
    }
  }
}

}  // namespace plansys2
