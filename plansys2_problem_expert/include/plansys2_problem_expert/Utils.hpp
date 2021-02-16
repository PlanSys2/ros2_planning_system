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
#include "plansys2_pddl_parser/Tree.h"

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
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions,
  bool apply = false,
  bool use_state = false,
  bool negate = false);

std::tuple<bool, bool, double> evaluate(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply = false);

std::tuple<bool, bool, double> evaluate(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions,
  bool apply = false);

/// Check a PDDL expression represented as a tree.
/**
* \param[in] node The root node of the PDDL expression.
* \param[in] problem_client The problem expert client.
* \return ret Truth value of the PDDL expression.
*
* This function calls the evaluate function.
*/
bool check(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client);

bool check(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions);

/// Apply a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \return success Indicates whether the execution was successful.
 *
 * This function calls the evaluate function.
 */
bool apply(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client);

bool apply(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions);

std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> get_subtrees(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node);

std::shared_ptr<parser::pddl::tree::DurativeAction> get_action_from_string(
  const std::string & action_expr,
  std::shared_ptr<plansys2::DomainExpertClient> domain_client);

std::vector<std::string> get_params(const std::string & action_expr);

std::string get_name(const std::string & action_expr);

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
