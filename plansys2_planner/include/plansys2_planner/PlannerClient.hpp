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

#ifndef PLANSYS2_PLANNER__PLANNERCLIENT_HPP_
#define PLANSYS2_PLANNER__PLANNERCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "plansys2_planner/PlannerInterface.hpp"

#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_plan_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @class plansys2::PlannerClient
 * @brief Client implementation of the PlannerInterface.
 *
 * This class provides a client that connects to the planner services
 * to request plans for PDDL planning problems.
 */
class PlannerClient : public PlannerInterface
{
public:
  /**
   * @brief Constructor for the PlannerClient.
   *
   * Creates a ROS node and configures service clients to connect to the planner services.
   * It waits for a plan as long as the planner plans (its plan_solver_timeout) plus a margin;
   * its own plan_solver_timeout parameter, if positive, sets that time instead.
   */
  PlannerClient();

  /**
   * @brief Generate a single plan for a PDDL planning problem.
   *
   * @param[in] domain PDDL domain definition as a string.
   * @param[in] problem PDDL problem definition as a string.
   * @param[in] node_namespace Namespace used for organizing temporary files and resources.
   * @return std::optional<plansys2_msgs::msg::Plan> Plan message containing the plan if found,
   *         std::nullopt if no plan could be generated.
   */
  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

  /**
   * @brief Generate multiple alternative plans for a PDDL planning problem.
   *
   * @param[in] domain PDDL domain definition as a string.
   * @param[in] problem PDDL problem definition as a string.
   * @param[in] node_namespace Namespace used for organizing temporary files and resources.
   * @return A message containing an array of plans found for the problem.
   */
  plansys2_msgs::msg::PlanArray getPlanArray(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

private:
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr
    get_plan_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlanArray>::SharedPtr
    get_plan_array_client_;

  // How long to wait for the planner's answer: its plan_solver_timeout and a margin
  std::chrono::nanoseconds answer_wait();

  rclcpp::Node::SharedPtr node_;
  // The planner's parameters: its plan_solver_timeout is how long it plans
  rclcpp::AsyncParametersClient::SharedPtr planner_parameters_;
  double planner_timeout_ {15.0};  // The last one read
};

}  // namespace plansys2

#endif  // PLANSYS2_PLANNER__PLANNERCLIENT_HPP_
