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

#ifndef PLANSYS2_KAIROS_PLAN_SOLVER__KAIROS_PLAN_SOLVER_HPP_
#define PLANSYS2_KAIROS_PLAN_SOLVER__KAIROS_PLAN_SOLVER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <list>
#include <set>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_msgs/msg/plan.hpp"

namespace kairos
{
class Domain;
}  // namespace kairos

namespace plansys2
{

/**
 * @class plansys2::KairosPlanSolver
 * @brief Plan solver that runs the Kairos planner in-process (no files and no child process), or
 * isolated in a process of its own.
 *
 * The last parsed domains are kept between calls, found by a hash of their text: PlanSys2
 * usually sends the same domain, but it may change it at runtime.
 *
 * Parameters, under the plugin name (e.g. KAIROS.timeout):
 * - timeout: seconds per plan, capped by the planner node timeout (0: only that one);
 * - anytime: keep improving the first plan;
 * - no_improvement: with anytime, seconds without a better plan before returning the best;
 * - search: M1 search preset ("auto", "lama", "ff", ...);
 * - engine: "auto", "m1", "m2" or "m3";
 * - memory_limit_mb: 0 for half of the available memory;
 * - threads: phases of the search at once (1: one after another, reproducible);
 * - log_level: Kairos messages forwarded to the ROS logger ("debug", "info", "warn", "error");
 * - dump_dir: when not empty, Kairos writes its intermediate structures there;
 * - isolated: run the kairos executable in a process of its own, as popf: a crash or a hang of
 *   Kairos does not take down the planner node, at the cost of a few ms per plan (process
 *   start, files, no domain cache); log_level and dump_dir do not apply;
 * - executable: the kairos executable for isolated (empty: the one installed with Kairos).
 */
class KairosPlanSolver : public PlanSolverBase
{
public:
  KairosPlanSolver() = default;

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override;

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "",
    const rclcpp::Duration solver_timeout = 15s) override;

  /// @brief Valid PDDL that Kairos can plan with.
  bool isDomainValid(
    const std::string & domain,
    const std::string & node_namespace = "") override;

  /// @brief All of PDDL, PDDL+ included.
  std::set<std::string> supported_requirements() const override;

private:
  std::string param(const std::string & name) const {return plugin_name_ + "." + name;}

  // Planning in a process of its own: the kairos executable, as popf
  std::optional<plansys2_msgs::msg::Plan> plan_isolated(
    const std::string & domain, const std::string & problem,
    const rclcpp::Duration & solver_timeout, std::chrono::milliseconds timeout);

  // The parsed domain, from the cache if possible, or nullptr (logs the errors)
  std::shared_ptr<const kairos::Domain> parse_domain(const std::string & domain);

  struct CachedDomain
  {
    size_t hash;
    std::string text;  // compared on equal hashes, so a collision never mixes domains
    std::shared_ptr<const kairos::Domain> domain;
  };
  static constexpr size_t kDomainCacheSize = 4;

  std::string plugin_name_;
  std::mutex domain_mutex_;
  std::list<CachedDomain> domains_;  // most recently used first
};

}  // namespace plansys2

#endif  // PLANSYS2_KAIROS_PLAN_SOLVER__KAIROS_PLAN_SOLVER_HPP_
