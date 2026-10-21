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
#include <memory>
#include <string>
#include <vector>

#include "plansys2_planner/PlannerInterface.hpp"

#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_plan_array.hpp"

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/ServiceCall.hpp"
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
   * @brief Creates the client on a node it does not own: the node of a PlanSys2 component or of
   * an application. It creates no node of its own.
   */
  explicit PlannerClient(ClientNode::Interfaces node);

  template<typename NodeT>
  explicit PlannerClient(const std::shared_ptr<NodeT> & node)
  : PlannerClient(ClientNode::Interfaces(*node)) {}

  /**
   * @brief Waits until its component is active, as every call does: false if ROS shuts down.
   */
  bool wait_active() {return active_->wait();}

  /**
   * @brief While interrupted, its calls give up at once instead of waiting: for the owner that
   * stops a thread which may be in a call.
   */
  void interrupt(bool interrupted) {node_->interrupt(interrupted);}

  /**
   * @brief How long a call waits for its component to be active before it gives up (zero, the
   * default: as long as it takes).
   */
  void set_max_wait(std::chrono::milliseconds max_wait) {active_->set_max_wait(max_wait);}

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
  ServiceClient<plansys2_msgs::srv::GetPlan>::SharedPtr
    get_plan_client_;
  ServiceClient<plansys2_msgs::srv::GetPlanArray>::SharedPtr
    get_plan_array_client_;

  // How long to wait for the planner's answer: its plan_solver_timeout and a margin
  std::chrono::nanoseconds answer_wait();

  std::shared_ptr<ClientNode> node_;
  // Its component: every call waits until it is active
  std::shared_ptr<ActiveComponent> active_;
  // The planner's parameters: its plan_solver_timeout is how long it plans
  rclcpp::AsyncParametersClient::SharedPtr planner_parameters_;
  double planner_timeout_ {15.0};  // The last one read
};

}  // namespace plansys2

#endif  // PLANSYS2_PLANNER__PLANNERCLIENT_HPP_
