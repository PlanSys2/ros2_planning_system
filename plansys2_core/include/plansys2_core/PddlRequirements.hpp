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

#ifndef PLANSYS2_CORE__PDDLREQUIREMENTS_HPP_
#define PLANSYS2_CORE__PDDLREQUIREMENTS_HPP_

#include <set>
#include <string>
#include <vector>

namespace plansys2
{

/**
 * @brief Every PDDL requirement PlanSys2 knows, as written in PDDL (":conditional-effects"),
 * and ":either-types" and ":timed-initial-fluents", which PDDL has no flag for.
 * @return The requirements.
 */
const std::set<std::string> & known_pddl_requirements();

/**
 * @brief The requirements a task uses, without parsing it.
 *
 * Those its syntax shows: :derived, when, or, forall, exists, either, preference, :constraints,
 * :process, :event, #t, ?duration inequalities, functions, negated conditions and timed initial
 * literals. Domains declare more than they use (:adl), so a declared requirement counts only if
 * the syntax cannot show it, as :action-costs.
 *
 * @param[in] domain The PDDL domain.
 * @param[in] problem The PDDL problem, if any.
 * @return The requirements, as written in PDDL.
 */
std::set<std::string> pddl_requirements(
  const std::string & domain, const std::string & problem = "");

/**
 * @brief The requirements in needed that supported lacks.
 * @param[in] needed What a task needs, from pddl_requirements.
 * @param[in] supported What a solver plans with; empty if it does not say.
 * @return The missing requirements, none if supported is empty.
 */
std::vector<std::string> missing_requirements(
  const std::set<std::string> & needed, const std::set<std::string> & supported);

}  // namespace plansys2

#endif  // PLANSYS2_CORE__PDDLREQUIREMENTS_HPP_
