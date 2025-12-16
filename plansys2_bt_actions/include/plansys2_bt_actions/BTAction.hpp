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

#ifndef PLANSYS2_BT_ACTIONS__BTACTION_HPP_
#define PLANSYS2_BT_ACTIONS__BTACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @class plansys2::BTAction
 * @brief Class that implements an action executor using Behavior Trees.
 *
 * This class provides a framework for executing PDDL actions using Behavior Trees.
 * It loads and executes a BT from an XML file and handles the lifecycle operations.
 */
class BTAction : public plansys2::ActionExecutorClient
{
public:
  /**
   * @brief Constructor for the BTAction.
   *
   * @param[in] action Name of the action this executor handles.
   */
  explicit BTAction(const std::string & action);

  /**
   * @brief Constructor for the BTAction.
   *
   * @param[in] action Name of the action this executor handles.
   * @param[in] rate Execution rate for the action.
   */
  [[deprecated("Use BTAction(const std::string & action) instead")]]
  BTAction(const std::string & action, const std::chrono::nanoseconds & rate);

  /**
   * @brief Get the name of the action.
   *
   * @return std::string The action name.
   */
  const std::string & getActionName() const {return action_;}

  /**
   * @brief Get the path to the BT XML file.
   *
   * @return std::string The path to the BT XML file.
   */
  const std::string & getBTFile() const {return bt_xml_file_;}

protected:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Configures the action executor.
   *
   * Retrieves parameters for action name, BT XML file, and plugins.
   * Registers BT plugins and creates a blackboard.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if configuration successful, FAILURE otherwise.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the action executor.
   *
   * Creates the BT from the XML file, sets up the blackboard with action arguments,
   * and initializes logging if enabled.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if activation successful, FAILURE otherwise.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the action executor.
   *
   * Cleans up loggers and halts the BT.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if deactivation successful, FAILURE otherwise.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up resources.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if cleanup successful, FAILURE otherwise.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Main work method that executes the BT.
   *
   * Called periodically to execute a tick of the BT.
   * Handles the BT execution status (SUCCESS, RUNNING, FAILURE)
   * and reports feedback/results accordingly.
   */
  void do_work();

  /**
   * @brief Add Groot2 monitor to publish BT status changes
   * @param tree BT to monitor
   * @param server_port Groot2 Server port, first of the pair (server_port, publisher_port)
   */
  void add_groot_monitoring(BT::Tree * tree, uint16_t server_port);

  /**
   * @brief Reset Groot2 monitor
   */
  void reset_groot_monitor();

  /**
   * @brief Factory for creating Behavior Trees instances.
   * @return BT::BehaviorTreeFactory& Reference to the factory.
   */
  BT::BehaviorTreeFactory factory_;

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string action_;
  std::string bt_xml_file_;
  std::vector<std::string> plugin_list_;
  bool finished_;
  std::unique_ptr<BT::FileLogger2> bt_file_logger_;
  std::unique_ptr<BT::MinitraceLogger> bt_minitrace_logger_;
  // Groot2 monitor
  std::unique_ptr<BT::Groot2Publisher> groot_monitor_;

  // Default timeout value while waiting for response from a server
  std::chrono::milliseconds default_server_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // Duration for each iteration of BT execution
  std::chrono::milliseconds bt_loop_duration_;
};

}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__BTACTION_HPP_
