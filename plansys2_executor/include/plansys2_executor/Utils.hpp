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

#ifndef PLANSYS2_EXECUTOR__UTILS_HPP_
#define PLANSYS2_EXECUTOR__UTILS_HPP_

#include <tuple>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_pddl_parser/Tree.h"

namespace plansys2
{

/// Check a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \return result <- tuple(bool, double)
 *         result(0) truth value of boolen expression
 *         result(1) value of numeric expression
 */
std::tuple<bool, double> check(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client);

/// Apply a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \param[in] negate Invert the truth value.
 * \return result <- tuple(bool, bool, double)
 *         result(0) true if success
 *         result(1) truth value of boolen expression
 *         result(2) value of numeric expression
 */
std::tuple<bool, bool, double> apply(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool negate = false);

std::shared_ptr<parser::pddl::tree::DurativeAction> get_action_from_string(
  const std::string & action_expr,
  std::shared_ptr<plansys2::DomainExpertClient> domain_client);

std::vector<std::string> get_params(const std::string & action_expr);

std::string get_name(const std::string & action_expr);

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__UTILS_HPP_
