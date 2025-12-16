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
#include <filesystem>
#include <iomanip>
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <chrono>

#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "std_msgs/msg/header.hpp"
#include "plansys2_bt_actions/BTAction.hpp"
#include "plansys2_bt_actions/BTUtils.hpp"
#include "plansys2_bt_actions/JSONUtils.hpp"

namespace plansys2
{

BTAction::BTAction(const std::string & action)
: ActionExecutorClient(action)
{
  declare_parameter<std::string>("bt_xml_file", "");
  declare_parameter<std::vector<std::string>>("plugins", std::vector<std::string>({}));
  declare_parameter<bool>("bt_file_logging", false);
  declare_parameter<bool>("bt_minitrace_logging", false);
  declare_parameter<bool>("enable_groot_monitoring", false);
  declare_parameter<int>("server_port", -1);
  declare_parameter<int>("server_timeout", 5000);
  declare_parameter<int>("wait_for_service_timeout", 1000);
}

BTAction::BTAction(const std::string & action, const std::chrono::nanoseconds & rate)
: ActionExecutorClient(action, rate)
{
  declare_parameter<std::string>("bt_xml_file", "");
  declare_parameter<std::vector<std::string>>("plugins", std::vector<std::string>({}));
  declare_parameter<bool>("bt_file_logging", false);
  declare_parameter<bool>("bt_minitrace_logging", false);
  declare_parameter<bool>("enable_groot_monitoring", false);
  declare_parameter<int>("server_port", -1);
  declare_parameter<int>("server_timeout", 5000);
  declare_parameter<int>("wait_for_service_timeout", 1000);
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

  int default_server_timeout;
  get_parameter("server_timeout", default_server_timeout);
  default_server_timeout_ = std::chrono::milliseconds(default_server_timeout);
  int wait_for_service_timeout;
  get_parameter("wait_for_service_timeout", wait_for_service_timeout);
  wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);
  bt_loop_duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(period_);

  BT::SharedLibrary loader;

  for (auto plugin : plugin_lib_names) {
    factory_.registerFromPlugin(loader.getOSName(plugin));
  }

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items in the blackboard
  blackboard_->set<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node", shared_from_this());
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
  blackboard_->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", wait_for_service_timeout_);
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);

  return ActionExecutorClient::on_configure(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  plugin_list_.clear();
  blackboard_.reset();
  return ActionExecutorClient::on_cleanup(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // If a new tree is created, than the Groot2 Publisher must be destroyed
  reset_groot_monitor();

  try {
    tree_ = factory_.createTreeFromFile(bt_xml_file_, blackboard_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Failed to create BT with exception: " << ex.what());
    RCLCPP_ERROR(get_logger(), "Transition to activate failed");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  for (int i = 0; i < get_arguments().size(); i++) {
    auto arg = get_arguments()[i];
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Setting arg" << i << " [" << arg << "]");
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, arg);
  }

  if (get_parameter("bt_file_logging").as_bool() ||
    get_parameter("bt_minitrace_logging").as_bool())
  {
    auto temp_path = std::filesystem::temp_directory_path();
    std::filesystem::path node_name_path = get_name();
    std::filesystem::create_directories(temp_path / node_name_path);

    auto now_time_t =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream filename;
    filename << "/tmp/" << get_name() << "/bt_trace_";
    filename << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d__%H_%M_%S");

    if (get_parameter("bt_file_logging").as_bool()) {
      std::string filename_extension = filename.str() + ".btlog";
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Logging to file: " << filename_extension);
      bt_file_logger_ =
        std::make_unique<BT::FileLogger2>(tree_, filename_extension.c_str());
    }

    if (get_parameter("bt_minitrace_logging").as_bool()) {
      std::string filename_extension = filename.str() + ".json";
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Logging to file: " << filename_extension);
      bt_minitrace_logger_ =
        std::make_unique<BT::MinitraceLogger>(tree_, filename_extension.c_str());
    }
  }

  bool enable_groot_monitoring = get_parameter("enable_groot_monitoring").as_bool();
  int server_port = get_parameter("server_port").as_int();
  if (enable_groot_monitoring) {
    if (server_port <= 0) {
      RCLCPP_WARN(get_logger(), "Groot2 monitoring port not provided, disabling it");
    } else {
      RCLCPP_INFO(get_logger(), "Enabling Groot2 monitoring on port: %d", server_port);
      add_groot_monitoring(&tree_, server_port);
    }
  }

  finished_ = false;
  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  bt_minitrace_logger_.reset();
  bt_file_logger_.reset();
  tree_.haltTree();
  reset_groot_monitor();

  return ActionExecutorClient::on_deactivate(previous_state);
}

void BTAction::do_work()
{
  if (!finished_) {
    BT::NodeStatus result;
    try {
      result = tree_.rootNode()->executeTick();
    } catch (BT::LogicError e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
      finish(false, 0.0, "BTAction behavior tree threw a BT::LogicError");
    } catch (BT::RuntimeError e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
      finish(false, 0.0, "BTAction behavior tree threw a BT::RuntimeError");
    } catch (std::exception e) {
      finish(false, 0.0, "BTAction behavior tree threw an unknown exception");
    }

    switch (result) {
      case BT::NodeStatus::SUCCESS:
        finish(true, 1.0, "BTAction behavior tree returned SUCCESS");
        finished_ = true;
        break;
      case BT::NodeStatus::RUNNING:
        send_feedback(0.0, "BTAction behavior tree returned RUNNING");
        break;
      case BT::NodeStatus::FAILURE:
        finish(false, 1.0, "BTAction behavior tree returned FAILURE");
        finished_ = true;
        break;
    }
  }
}

void BTAction::add_groot_monitoring(BT::Tree * tree, uint16_t server_port)
{
  // This logger publish status changes using Groot2
  groot_monitor_ = std::make_unique<BT::Groot2Publisher>(*tree, server_port);

  // Register common types JSON definitions
  BT::RegisterJsonDefinition<builtin_interfaces::msg::Time>();
  BT::RegisterJsonDefinition<std_msgs::msg::Header>();
}

void BTAction::reset_groot_monitor()
{
  if (groot_monitor_) {
    groot_monitor_.reset();
  }
}

}  // namespace plansys2
