// Copyright 2020 Intelligent Robotics Lab
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

#ifndef PLANSYS2_EXECUTOR__COMPUTEBT_HPP_
#define PLANSYS2_EXECUTOR__COMPUTEBT_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pluginlib/class_loader.hpp"

namespace plansys2
{

/**
 * @class plansys::ComputeBT
 * @brief ROS2 Lifecycle Node that generates behavior trees from plans.
 *
 * This node provides a service to transform a PDDL plan into a behavior tree (BT)
 * representation that can be executed. It loads BT templates for actions, starts
 * the required planning services, and handles the conversion process.
 */
class ComputeBT : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor for the ComputeBT node.
   */
  ComputeBT();

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

private:
  /**
   * @brief Service callback for computing a behavior tree.
   *
   * Loads domain and problem files, generates a plan, transforms it into a behavior tree,
   * and saves the results to files. Also publishes the dot graph representation.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty trigger request.
   * @param[out] response Service response indicating success or failure.
   */
  void computeBTCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Reads a problem file and returns its contents as a string.
   *
   * @param[in] filename Path to the problem file.
   * @return The contents of the problem file as a string.
   */
  std::string getProblem(const std::string & filename) const;

  /**
   * @brief Saves a plan to a file.
   *
   * Creates a file named [filename]_plan.pddl and writes the plan details.
   *
   * @param[in] plan The plan to save.
   * @param[in] filename Base name for the output file (without extension).
   */
  void savePlan(const plansys2_msgs::msg::Plan & plan, const std::string & filename) const;

  /**
   * @brief Saves a behavior tree XML to a file.
   *
   * Creates a file named [filename]_bt.xml and writes the behavior tree XML.
   *
   * @param[in] bt_xml The behavior tree XML to save.
   * @param[in] filename Base name for the output file (without extension).
   */
  void saveBT(const std::string & bt_xml, const std::string & filename) const;

  /**
   * @brief Saves a dot graph representation to a file.
   *
   * Creates a file named [filename]_graph.dot and writes the dot graph.
   *
   * @param[in] dotgraph The dot graph representation to save.
   * @param[in] filename Base name for the output file (without extension).
   */
  void saveDotGraph(const std::string & dotgraph, const std::string & filename) const;

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

  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::PlannerNode> planner_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr compute_bt_srv_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr dotgraph_pub_;

  // Groot2 monitor
  std::unique_ptr<BT::Groot2Publisher> groot_monitor_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__COMPUTEBT_HPP_
