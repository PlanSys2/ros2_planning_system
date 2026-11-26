// Copyright 2026 Intelligent Robotics Lab
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

#ifndef PLANSYS2_KLIMAX_PLAN_SOLVER__KLIMAX_PLAN_SOLVER_HPP_
#define PLANSYS2_KLIMAX_PLAN_SOLVER__KLIMAX_PLAN_SOLVER_HPP_

#include <optional>
#include <set>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2
{

/**
 * @class plansys2::KlimaxPlanSolver
 * @brief Plan solver for hierarchical (HTN) tasks in HDDL, with the Klimax planner in-process.
 *
 * The plan has its actions in `items`, as any other, and its decomposition in `tasks`: from the
 * initial task network down to the actions, every task with its method, the order of its
 * subtasks and the precondition of the method. The HTNBTBuilder executes it so.
 *
 * Parameters, under the plugin name (e.g. KLIMAX.timeout):
 * - timeout: seconds per plan, capped by the planner node timeout (0: only that one);
 * - anytime: go on after the first plan, looking for better ones until the timeout (or until
 *   one is proved the best).
 */
class KlimaxPlanSolver : public PlanSolverBase
{
public:
  KlimaxPlanSolver() = default;

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override;

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "",
    const rclcpp::Duration solver_timeout = 15s) override;

  /// @brief A valid HDDL domain.
  bool isDomainValid(
    const std::string & domain,
    const std::string & node_namespace = "") override;

  /// @brief HDDL: hierarchical tasks, with what PDDL has.
  std::set<std::string> supported_requirements() const override;

private:
  std::string param(const std::string & name) const {return plugin_name_ + "." + name;}

  std::string plugin_name_;
};

}  // namespace plansys2

#endif  // PLANSYS2_KLIMAX_PLAN_SOLVER__KLIMAX_PLAN_SOLVER_HPP_
