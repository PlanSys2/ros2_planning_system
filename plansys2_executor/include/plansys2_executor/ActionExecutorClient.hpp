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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_

#include <string>
#include <memory>
#include <vector>

#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_performer_status.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class plansys2::ActionExecutorClient
 * @brief Base class for implementing action executors in the planning system.
 *
 * ActionExecutorClient provides a framework for executing PDDL actions in the real world.
 * It handles the communication with the executor node, manages the action lifecycle,
 * and provides feedback about the execution progress.
 */
class ActionExecutorClient : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  using Ptr = std::shared_ptr<ActionExecutorClient>;

  /**
   * @brief Factory method to create a shared pointer to an ActionExecutorClient.
   *
   * @param[in] node_name Name for the ROS node.
   * @return Shared pointer to the newly created ActionExecutorClient.
   */
  static Ptr make_shared(const std::string & node_name)
  {
    return std::make_shared<ActionExecutorClient>(node_name);
  }

  /**
   * @brief Constructor for the ActionExecutorClient.
   *
   * @param[in] node_name Name for the ROS node.
   */
  explicit ActionExecutorClient(const std::string & node_name);

  /**
   * @brief Constructor for the ActionExecutorClient.
   *
   * @param[in] node_name Name for the ROS node.
   * @param[in] rate Execution rate for periodic work.
   */
  [[deprecated("Use ActionExecutorClient(const std::string & node_name) instead")]]
  ActionExecutorClient(const std::string & node_name, const std::chrono::nanoseconds & rate);

  /**
   * @brief Get the time when the action execution started.
   *
   * @return rclcpp::TimeThe ROS time when action execution was activated.
   */
  rclcpp::Time get_start_time() const {return start_time_;}

  /**
   * @brief Get the current internal status of the action executor.
   *
   * @return plansys2_msgs::msg::ActionPerformerStatus Current status message with state, timestamp,
   *         action and specialized arguments.
   */
  plansys2_msgs::msg::ActionPerformerStatus get_internal_status() const {return status_;}

protected:
  /**
   * @brief Main work method that derived classes should implement.
   *
   * This method is called periodically at the rate specified in the constructor.
   * Derived classes should implement this method to perform the actual execution
   * of the action and report progress.
   */
  virtual void do_work() {}

  /**
   * @brief Get the arguments for the current action execution.
   *
   * @return Reference to the vector of arguments.
   */
  const std::vector<std::string> & get_arguments() const {return current_arguments_;}

  /**
   * @brief Get the name of the action this executor manages.
   *
   * @return std::stringThe action name.
   */
  const std::string get_action_name() const {return action_managed_;}

  /**
   * @brief Configures the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if configuration is successful, FAILURE otherwise.
   */
  virtual CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if activation is successful, FAILURE otherwise.
   */
  virtual CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if deactivation is successful, FAILURE otherwise.
   */
  virtual CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Process messages from the actions hub.
   *
   * Handles REQUEST, CONFIRM, REJECT, and CANCEL messages, managing the
   * action execution lifecycle accordingly.
   *
   * @param[in] msg The action execution message received.
   */
  void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  /**
   * @brief Check if this executor should execute the given action.
   *
   * Verifies that the action name matches and that any specialized arguments
   * are compatible with the requested arguments.
   *
   * @param[in] action The action name to check.
   * @param[in] args The arguments to check.
   * @return True if this executor should handle the action, false otherwise.
   */
  bool should_execute(const std::string & action, const std::vector<std::string> & args);

  /**
   * @brief Send a response to a request message.
   *
   * Indicates that this executor is willing to execute the requested action.
   *
   * @param[in] msg The original request message.
   */
  void send_response(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  /**
   * @brief Send feedback about the action execution progress.
   *
   * @param[in] completion Percentage of completion (0.0 to 1.0).
   * @param[in] status Optional status message describing the current state.
   */
  void send_feedback(float completion, const std::string & status = "");

  /**
   * @brief Finish the action execution.
   *
   * Deactivates the executor if active and sends a FINISH message to
   * indicate completion or failure.
   *
   * @param[in] success Whether the action was successful.
   * @param[in] completion Final completion percentage (typically 1.0 for success).
   * @param[in] status Optional status message describing the final state.
   */
  void finish(bool success, float completion, const std::string & status = "");

  // Period for the timer to call do_work
  std::chrono::nanoseconds period_;
  std::string action_managed_;
  bool committed_;

  std::vector<std::string> current_arguments_;
  std::vector<std::string> specialized_arguments_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecution>::SharedPtr
    action_hub_pub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_hub_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionPerformerStatus>::SharedPtr
    status_pub_;
  rclcpp::TimerBase::SharedPtr hearbeat_pub_;
  plansys2_msgs::msg::ActionPerformerStatus status_;
  rclcpp::Time start_time_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
