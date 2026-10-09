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

#ifndef PLANSYS2_CORE__PLANSOLVERBASE_HPP_
#define PLANSYS2_CORE__PLANSOLVERBASE_HPP_

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include "plansys2_msgs/msg/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace plansys2
{

/**
 * @class plansys2::PlanSolverBase
 * @brief Abstract base class for plan solvers in PlanSys2.
 *
 * This class defines the interface for plan solver plugins, including methods for
 * configuration, plan generation, domain validation, and planner execution.
 */
class PlanSolverBase
{
public:
  using Ptr = std::shared_ptr<plansys2::PlanSolverBase>;

  /**
   * @brief Default constructor.
   */
  PlanSolverBase() {}

  /**
   * @brief Virtual destructor, plugins are destroyed through this base class.
   */
  virtual ~PlanSolverBase() = default;

  /**
   * @brief Configure the plan solver with a lifecycle node and plugin name.
   * @param[in] lc_node Shared pointer to the lifecycle node.
   * @param[in] plugin_name The name of the plugin.
   */
  virtual void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, const std::string & plugin_name) = 0;

  /**
   * @brief Generate a plan given a PDDL domain and problem definition.
   * @param[in] domain The PDDL domain as a string.
   * @param[in] problem The PDDL problem definition as a string.
   * @param[in] node_namespace The node namespace (optional).
   * @param[in] solver_timeout Timeout for the solver (optional, default: 15s).
   * @return std::optional<plansys2_msgs::msg::Plan> The resulting plan if found, otherwise empty.
   */
  virtual std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "", const rclcpp::Duration solver_timeout = 15s) = 0;

  /**
   * @brief Validate a PDDL domain.
   * @param[in] domain The PDDL domain as a string.
   * @param[in] node_namespace The node namespace (optional).
   * @return true if the domain is valid, false otherwise.
   */
  virtual bool isDomainValid(
    const std::string & domain, const std::string & node_namespace = "") = 0;

  /**
   * @brief Request cancellation of the current planning process.
   */
  virtual void cancel() {cancel_requested_ = true;}

  /**
   * @brief Execute the planner with a command.
   *
   * The command runs in its own process group with its standard output written to
   * plan_path. On timeout or cancel() the whole group is killed, so processes it
   * started (e.g. a planner under `ros2 run`) do not outlive the call.
   *
   * @param[in] command The command to execute the planner.
   * @param[in] solver_timeout Timeout for the solver.
   * @param[in] plan_path Path to store the resulting plan.
   * @return true if the planner exited with status 0 and its output was fully
   *         written, false on timeout, cancellation, signal, non-zero exit or error.
   */
  virtual bool execute_planner(
    const std::string & command, const rclcpp::Duration & solver_timeout,
    const std::string & plan_path);

protected:
  // Lifecycle node pointer.
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  // Flag indicating if cancellation was requested.
  std::atomic<bool> cancel_requested_{false};

  /**
   * @brief Tokenize a command string.
   * @param[in] command The command string to tokenize.
   * @return char** Array of C-style strings representing the tokens.
   */
  char ** tokenize(const std::string & command);
};

}  // namespace plansys2

#endif  // PLANSYS2_CORE__PLANSOLVERBASE_HPP_
