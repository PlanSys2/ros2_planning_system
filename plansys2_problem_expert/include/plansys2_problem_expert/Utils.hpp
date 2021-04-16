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

#ifndef PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
#define PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_

#include <tuple>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <set>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

/// Evaluate a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \param[in] predicates Current predicates state.
 * \param[in] functions Current functions state.
 * \param[in] apply Apply result to problem expert or state.
 * \param[in] use_state Use state representation or problem client.
 * \param[in] negate Invert the truth value.
 * \return result <- tuple(bool, bool, double)
 *         result(0) true if success
 *         result(1) truth value of boolen expression
 *         result(2) value of numeric expression
 */
std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<plansys2_msgs::msg::Node> & predicates,
  std::vector<plansys2_msgs::msg::Node> & functions,
  bool apply = false,
  bool use_state = false,
  uint8_t node_id = 0,
  bool negate = false);

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply = false,
  uint32_t node_id = 0);

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2_msgs::msg::Node> & predicates,
  std::vector<plansys2_msgs::msg::Node> & functions,
  bool apply = false,
  uint32_t node_id = 0);

/// Check a PDDL expression represented as a tree.
/**
* \param[in] node The root node of the PDDL expression.
* \param[in] problem_client The problem expert client.
* \return ret Truth value of the PDDL expression.
*
* This function calls the evaluate function.
*/
bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id = 0);

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2_msgs::msg::Node> & predicates,
  std::vector<plansys2_msgs::msg::Node> & functions,
  uint32_t node_id = 0);

/// Apply a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \return success Indicates whether the execution was successful.
 *
 * This function calls the evaluate function.
 */
bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id = 0);

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2_msgs::msg::Node> & predicates,
  std::vector<plansys2_msgs::msg::Node> & functions,
  uint32_t node_id = 0);

/// Parse the action name from a plan action expression.
/**
 * \param[in] action_expr An action expression taken from a plan.
 * \return The name of the action.
 *
 * A plan action expression will have the following format.
 *   (<name> <param_1> ... <param_n>):<time>
 */
std::string get_action_name(
  const std::string & action_expr);

/// Parse the action parameter names from a plan action expression.
/**
 * \param[in] action_expr An action expression taken from a plan.
 * \return A list of the action parameter names.
 *
 * A plan action expression will have the following format.
 *   (<name> <param_1> ... <param_n>):<time>
 */
std::vector<std::string> get_action_params(
  const std::string & action_expr);

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
