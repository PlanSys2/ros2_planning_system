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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_

#include <optional>
#include <memory>

#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

#include "std_msgs/msg/string.hpp"
#include "plansys2_msgs/srv/get_domain_name.hpp"
#include "plansys2_msgs/srv/get_domain_types.hpp"
#include "plansys2_msgs/srv/get_domain_actions.hpp"
#include "plansys2_msgs/srv/get_domain_action_details.hpp"
#include "plansys2_msgs/srv/get_domain_derived_predicate_details.hpp"
#include "plansys2_msgs/srv/get_domain_durative_action_details.hpp"
#include "plansys2_msgs/srv/get_domain.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"
#include "plansys2_msgs/srv/validate_domain.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

/**
 * @class plansys2::DomainExpertNode
 * @brief ROS2 Lifecycle Node that manages the domain model and handles requests from DomainExpertClient.
 *
 * This node provides services to query and manipulate the planning domain,
 * including actions, predicates, functions, and derived predicates.
 */
class DomainExpertNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct a new DomainExpertNode object.
   * @param[in] options Node options for configuring the node.
   */
  explicit DomainExpertNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
   * @brief Service callback to retrieve the PDDL domain name.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the domain name and success status.
   */
  void get_domain_name_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainName::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainName::Response> response);

  /**
   * @brief Service callback to retrieve all types defined in the domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of types and success status.
   */
  void get_domain_types_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Response> response);

  /**
   * @brief Service callback to retrieve all non-durative actions in the domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of actions and success status.
   */
  void get_domain_actions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response);

  /**
   * @brief Service callback to retrieve details of a specific action.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the action name and parameters.
   * @param[out] response Service response containing the action details and success status.
   */
  void get_domain_action_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Response> response);

  /**
   * @brief Service callback to retrieve all durative actions in the domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of durative actions and success status.
   */
  void get_domain_durative_actions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response);

  /**
   * @brief Service callback to retrieve details of a specific durative action.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the durative action name and parameters.
   * @param[out] response Service response containing the durative action details and success status.
   */
  void get_domain_durative_action_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainDurativeActionDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainDurativeActionDetails::Response> response);

  /**
   * @brief Service callback to retrieve all predicates defined in the domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of predicates and success status.
   */
  void get_domain_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  /**
   * @brief Service callback to retrieve details of a specific predicate.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the predicate expression.
   * @param[out] response Service response containing the predicate details and success status.
   */
  void get_domain_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  /**
   * @brief Service callback to retrieve all functions defined in the domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of functions and success status.
   */
  void get_domain_functions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  /**
   * @brief Service callback to retrieve details of a specific function.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the function expression.
   * @param[out] response Service response containing the function details and success status.
   */
  void get_domain_function_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  /**
   * @brief Service callback to retrieve all derived predicates in the domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of derived predicates and success status.
   */
  void get_domain_derived_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Response> response);

  /**
   * @brief Service callback to retrieve details of a specific derived predicate.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the derived predicate name.
   * @param[out] response Service response containing the predicate rules and success status.
   */
  void get_domain_derived_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Response> response);

  /**
   * @brief Service callback to retrieve the complete PDDL domain.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the PDDL domain string and success status.
   */
  void get_domain_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomain::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomain::Response> response);

private:
  std::shared_ptr<DomainExpert> domain_expert_;

  rclcpp::Service<plansys2_msgs::srv::GetDomainName>::SharedPtr get_name_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainTypes>::SharedPtr get_types_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_domain_actions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainActionDetails>::SharedPtr
    get_domain_action_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainActions>::SharedPtr
    get_domain_durative_actions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainDurativeActionDetails>::SharedPtr
    get_domain_durative_action_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_domain_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_domain_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_domain_functions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_domain_function_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>::SharedPtr
    get_domain_derived_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>::SharedPtr
    get_domain_derived_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomain>::SharedPtr get_domain_service_;

  rclcpp::Client<plansys2_msgs::srv::ValidateDomain>::SharedPtr
    validate_domain_client_;
  rclcpp::CallbackGroup::SharedPtr validate_domain_callback_group_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr domain_pub_;

  std::unique_ptr<plansys2::POPFPlanSolver> popf_plan_solver_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_
