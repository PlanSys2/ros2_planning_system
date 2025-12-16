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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_

#include <memory>

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include "std_msgs/msg/empty.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/problem.hpp"
#include "plansys2_msgs/srv/affect_node.hpp"
#include "plansys2_msgs/srv/affect_param.hpp"
#include "plansys2_msgs/srv/add_problem.hpp"
#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/exist_node.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"
#include "plansys2_msgs/srv/is_problem_goal_satisfied.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_msgs/srv/clear_problem_knowledge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

/**
 * @class plansys2::ProblemExpertNode
 * @brief ROS 2 Lifecycle node that manages the PDDL problem representation and operations.
 *
 * This node provides services to add, remove, and query PDDL problem elements such as
 * instances, predicates, functions, and goals. It also publishes the problem state
 * and change notifications.
 */
class ProblemExpertNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the ProblemExpertNode.
   *
   * Initializes the node, declares parameters, and creates service servers and publishers.
   */
  explicit ProblemExpertNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
   * @brief Converts the current problem knowledge to a message.
   *
   * @return plansys2_msgs::msg::Knowledge::SharedPtr The knowledge message containing
   * the current problem state, including instances, predicates, functions, and goal.
   */
  plansys2_msgs::msg::Knowledge::SharedPtr get_knowledge_as_msg() const;

  /**
   * @brief Service callback to add a PDDL problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the PDDL problem string.
   * @param[out] response Service response with success status and error information.
   */
  void add_problem_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblem::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblem::Response> response);

  /**
   * @brief Service callback to add a problem goal.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the goal as a Tree.
   * @param[out] response Service response with success status and error information.
   */
  void add_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response);

  /**
   * @brief Service callback to add a problem instance.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the instance parameter.
   * @param[out] response Service response with success status and error information.
   */
  void add_problem_instance_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response);

  /**
   * @brief Service callback to add a problem predicate.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the predicate node.
   * @param[out] response Service response with success status and error information.
   */
  void add_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief Service callback to add a problem function.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the function node.
   * @param[out] response Service response with success status and error information.
   */
  void add_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief Service callback to get the problem goal.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the goal tree and success status.
   */
  void get_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response);

  /**
   * @brief Service callback to get details about a specific problem instance.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the instance name.
   * @param[out] response Service response with instance details and success status.
   */
  void get_problem_instance_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response);

  /**
   * @brief Service callback to get all problem instances.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of instances and success status.
   */
  void get_problem_instances_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Response> response);

  /**
   * @brief Service callback to get details about a specific problem predicate.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the predicate expression.
   * @param[out] response Service response with predicate details and success status.
   */
  void get_problem_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  /**
   * @brief Service callback to get all problem predicates.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of predicates and success status.
   */
  void get_problem_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  /**
   * @brief Service callback to get details about a specific problem function.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the function expression.
   * @param[out] response Service response with function details and success status.
   */
  void get_problem_function_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  /**
   * @brief Service callback to get all problem functions.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the list of states.
   */
  void get_problem_functions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  /**
   * @brief Service callback to get the full PDDL problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the problem.
   */
  void get_problem_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response);

  /**
   * @brief Service callback to check if a problem goal is satisfied.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the goal tree to check.
   * @param[out] response Service response with satisfaction status.
   */
  void is_problem_goal_satisfied_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Response> response);

  /**
   * @brief Service callback to remove the problem goal.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response with success status and error information.
   */
  void remove_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response);

  /**
   * @brief Service callback to clear all problem knowledge.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response with success status and error information.
   */
  void clear_problem_knowledge_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Response> response);

  /**
   * @brief Service callback to remove a problem instance.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the instance parameter to remove.
   * @param[out] response Service response with success status and error information.
   */
  void remove_problem_instance_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response);

  /**
   * @brief Service callback to remove a problem predicate.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the predicate node to remove.
   * @param[out] response Service response with success status and error information.
   */
  void remove_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief Service callback to remove a problem function.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the function node to remove.
   * @param[out] response Service response with success status and error information.
   */
  void remove_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief Service callback to check if a predicate exists in the problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the predicate node to check.
   * @param[out] response Service response with existence status.
   */
  void exist_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response);

  /**
   * @brief Service callback to check if a function exists in the problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the function node to check.
   * @param[out] response Service response with existence status.
   */
  void exist_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response);

  /**
   * @brief Service callback to update a problem function's value.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing the function node with updated value.
   * @param[out] response Service response with success status and error information.
   */
  void update_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

private:
  std::shared_ptr<ProblemExpert> problem_expert_;

  // Service servers
  rclcpp::Service<plansys2_msgs::srv::AddProblem>::SharedPtr
    add_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::AddProblemGoal>::SharedPtr
    add_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectParam>::SharedPtr
    add_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemGoal>::SharedPtr
    get_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
    get_problem_instance_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
    get_problem_instances_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_function_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_functions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblem>::SharedPtr
    get_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::IsProblemGoalSatisfied>::SharedPtr
    is_problem_goal_satisfied_service_;
  rclcpp::Service<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr
    remove_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::ClearProblemKnowledge>::SharedPtr
    clear_problem_knowledge_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectParam>::SharedPtr
    remove_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    update_problem_function_service_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Problem>::SharedPtr problem_pub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
