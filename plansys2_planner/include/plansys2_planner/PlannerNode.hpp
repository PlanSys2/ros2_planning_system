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

#ifndef PLANSYS2_PLANNER__PLANNERNODE_HPP_
#define PLANSYS2_PLANNER__PLANNERNODE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_core/PlanSolverBase.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_plan_array.hpp"
#include "plansys2_msgs/srv/validate_domain.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pluginlib/class_loader.hpp"

namespace plansys2
{

/**
 * @class plansys2::PlannerNode
 * @brief ROS2 Lifecycle Node that manages the planning system and handles planning requests.
 *
 * This node loads planner plugins based on configuration parameters, provides services
 * to generate plans from PDDL domains and problems, and validates domains.
 */
class PlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the PlannerNode.
   */
  explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the PlannerNode.
   */
  ~PlannerNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  using SolverMap = std::unordered_map<std::string, plansys2::PlanSolverBase::Ptr>;

  /**
   * @brief Configures the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if configuration is successful, FAILURE otherwise.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if activation is successful, FAILURE otherwise.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if deactivation is successful, FAILURE otherwise.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if cleanup is successful, FAILURE otherwise.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if shutdown is successful, FAILURE otherwise.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if error handling is successful, FAILURE otherwise.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Service callback to generate a plan for a PDDL problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing domain and problem PDDL strings.
   * @param[out] response Service response containing the generated plan and success status.
   */
  void get_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

  /**
   * @brief Service callback to generate multiple plans for a PDDL problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing domain and problem PDDL strings.
   * @param[out] response Service response containing an array of plans and success status.
   */
  void get_plan_array_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlanArray::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlanArray::Response> response);

  /**
   * @brief Service callback to validate a PDDL domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the domain PDDL string.
   * @param[out] response Service response with validation result and error information.
   */
  void validate_domain_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Response> response);

  /**
   * @brief Generate multiple plans for a PDDL problem using all configured planners.
   *
   * @param[in] domain PDDL domain string.
   * @param[in] problem PDDL problem string.
   * @return An array of plans found by the planners.
   */
  plansys2_msgs::msg::PlanArray get_plan_array(
    const std::string & domain, const std::string & problem);

  /**
   * @brief Set the timeout for plan solvers.
   *
   * @param[in] solver_timeout The new timeout duration.
   */
  void set_timeout(rclcpp::Duration solver_timeout) {solver_timeout_ = solver_timeout;}

private:
  pluginlib::ClassLoader<plansys2::PlanSolverBase> lp_loader_;
  SolverMap solvers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> solver_ids_;
  std::vector<std::string> solver_types_;
  rclcpp::Duration solver_timeout_;

  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  rclcpp::Service<plansys2_msgs::srv::GetPlanArray>::SharedPtr get_plan_array_service_;
  rclcpp::Service<plansys2_msgs::srv::ValidateDomain>::SharedPtr validate_domain_service_;
};

/**
 * @brief Declare a parameter if it has not been declared yet.
 *
 * @tparam NodeT Type of the ROS2 node.
 * @param[in] node Node to declare the parameter on.
 * @param[in] param_name Name of the parameter to declare.
 * @param[in] default_value Default value for the parameter.
 * @param[in] parameter_descriptor Parameter descriptor for additional metadata.
 */
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value = rclcpp::ParameterValue(),
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * @brief Get the plugin type parameter for a given plugin name.
 *
 * @tparam NodeT Type of the ROS2 node.
 * @param[in] node Node to get the parameter from.
 * @param[in] plugin_name Name of the plugin.
 * @return String containing the plugin type.
 */
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::ParameterValue(""));
  std::string plugin_type;
  if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
    RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }
  return plugin_type;
}

/**
 * @brief Get the arguments parameter for a given plugin name.
 *
 * @tparam NodeT Type of the ROS2 node.
 * @param[in] node Node to get the parameter from.
 * @param[in] plugin_name Name of the plugin.
 * @return String containing the plugin arguments.
 */
template<typename NodeT>
std::string get_args_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".args", rclcpp::ParameterValue(""));
  std::string plugin_arg;
  if (!node->get_parameter(plugin_name + ".args", plugin_arg)) {
    RCLCPP_FATAL(node->get_logger(), "'args' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }
  return plugin_arg;
}

}  // namespace plansys2

#endif  // PLANSYS2_PLANNER__PLANNERNODE_HPP_
