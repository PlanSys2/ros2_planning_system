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

#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "plansys2_bt_actions/BTAction.hpp"

namespace plansys2
{

BTAction::BTAction(
  const std::string & action,
  const std::chrono::nanoseconds & rate)
: ActionExecutorClient(action, rate)
{
  declare_parameter<std::string>("bt_xml_file", "");
  declare_parameter<std::vector<std::string>>(
    "plugins", std::vector<std::string>({}));
  declare_parameter<bool>("bt_file_logging", false);
  declare_parameter<bool>("bt_minitrace_logging", false);
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

  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", shared_from_this());

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
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, get_arguments()[i]);
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
      std::string filename_extension = filename.str() + ".fbl";
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Logging to file: " << filename_extension);
      bt_file_logger_ =
        std::make_unique<BT::FileLogger>(tree_, filename_extension.c_str());
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

#ifdef ZMQ_FOUND
  bool enable_groot_monitoring = get_parameter("enable_groot_monitoring").as_bool();
  int publisher_port = get_parameter("publisher_port").as_int();
  int server_port = get_parameter("server_port").as_int();
  unsigned int max_msgs_per_second = get_parameter("max_msgs_per_second").as_int();

  if (enable_groot_monitoring) {
    if (publisher_port <= 0 || server_port <= 0) {
      RCLCPP_WARN(
        get_logger(),
        "[%s] Groot monitoring ports not provided, disabling Groot monitoring."
        " publisher port: %d, server port: %d",
        get_name(), publisher_port, server_port);
    } else {
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
  }
#endif

  finished_ = false;
  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  publisher_zmq_.reset();
  bt_minitrace_logger_.reset();
  bt_file_logger_.reset();
  tree_.haltTree();

  return ActionExecutorClient::on_deactivate(previous_state);
}

void
BTAction::do_work()
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

}  // namespace plansys2
