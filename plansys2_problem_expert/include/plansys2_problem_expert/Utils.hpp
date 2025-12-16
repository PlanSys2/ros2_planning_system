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
#include <utility>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

/**
 * @brief Evaluate a PDDL expression represented as a tree.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in] problem_client The problem expert client.
 * @param[in,out] predicates Current predicates state.
 * @param[in,out] functions Current functions state.
 * @param[in] apply Apply result to problem expert or state (default: false).
 * @param[in] use_state Use state representation or problem client (default: false).
 * @param[in] node_id Node identifier in the tree (default: 0).
 * @param[in] negate Invert the truth value of the expression (default: false).
 * @return tuple(bool, bool, double) with execution result in this format:
 *         result <- tuple(bool, bool, double)
 *         result(0) true if success
 *         result(1) truth value of boolean expression
 *         result(2) value of numeric expression
 */
std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply = false,
  bool use_state = false,
  uint8_t node_id = 0,
  bool negate = false);

/**
 * @brief Evaluate a PDDL expression represented as a tree using only the problem client.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in] problem_client The problem expert client.
 * @param[in] apply Apply result to problem expert (default: false).
 * @param[in] node_id Node identifier in the tree.
 * @return tuple(bool, bool, double) with execution result.
 */
std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply = false,
  uint32_t node_id = 0);

/**
 * @brief Evaluate a PDDL expression represented as a tree using only local state.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in,out] predicates Current predicates state.
 * @param[in,out] functions Current functions state.
 * @param[in] apply Apply result to state (default: false).
 * @param[in] node_id Node identifier in the tree (default: 0).
 * @return tuple(bool, bool, double) with execution result.
 */
std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply = false,
  uint32_t node_id = 0);

/**
 * @brief Check a PDDL expression represented as a tree.
 *        This function calls the evaluate function.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in] problem_client The problem expert client.
 * @param[in] node_id Node identifier in the tree (default: 0).
 * @return bool Truth value of the PDDL expression.
 */
bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id = 0);

/**
 * @brief Check a PDDL expression represented as a tree using local state.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in] predicates Current predicates state.
 * @param[in] functions Current functions state.
 * @param[in] node_id Node identifier in the tree (default: 0).
 * @return bool Truth value of the PDDL expression.
 */
bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id = 0);

/**
 * @brief Apply a PDDL expression represented as a tree.
 *        This function calls the evaluate function.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in] problem_client The problem expert client.
 * @param[in] node_id Node identifier in the tree.
 * @return bool Indicates whether the execution was successful.
 */
bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id = 0);

/**
 * @brief Apply a PDDL expression represented as a tree using local state.
 * This function applies the effects of the PDDL expression to the local state.
 *
 * @param[in] tree The root node of the PDDL expression.
 * @param[in,out] predicates Current predicates state.
 * @param[in,out] functions Current functions state.
 * @param[in] node_id Node identifier in the tree (default: 0).
 * @return bool Indicates whether the execution was successful.
 */
bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id = 0);

/**
 * @brief Parse the action expression and time (optional) from an input string.

 *
 * @param[in] input The input string. It can have one of these formats:
 *                  "(<name> <param_1> ... <param_n>)"
 *                  "(<name> <param_1> ... <param_n>):<time>"
 * @return std::pair<std::string, int> with the action expression and start time.
 *         result <- pair(string, int)
*          result(0) The action expression.
*          result(1) The action start time.
 */
std::pair<std::string, int> parse_action(const std::string & input);

/**
 * @brief Parse the action expression from an input string.
 *
 * @param[in] input The input string. It can have one of these formats:
 *                  "(<name> <param_1> ... <param_n>)"
 *                  "(<name> <param_1> ... <param_n>):<time>"
 * @return std::string The action expression.
 */
std::string get_action_expression(const std::string & input);

/**
 * @brief Parse the action time from an input string.
 *
 * @param[in] input The input string. It can have one of these formats:
 *                  "(<name> <param_1> ... <param_n>)"
 *                  "(<name> <param_1> ... <param_n>):<time>"
 * @return int The action start time.
 */
int get_action_time(const std::string & input);

/**
 * @brief Parse the action name from an input string.
 *
 * @param[in] input The input string. It can have one of these formats:
 *                  "(<name> <param_1> ... <param_n>)"
 *                  "(<name> <param_1> ... <param_n>):<time>"
 * @return std::string The name of the action.
 */
std::string get_action_name(const std::string & input);

/**
 * @brief Parse the action parameter names from an input string.
 *
 * @param[in] action_expr The input string. It can have one of these formats:
 *                        "(<name> <param_1> ... <param_n>)"
 *                        "(<name> <param_1> ... <param_n>):<time>"
 * @return std::vector<std::string> A vector containing the names of the parameters.
 */
std::vector<std::string> get_action_params(const std::string & action_expr);

/**
 * @brief Replace parameter names in children nodes of a PDDL expression tree.
 * This function creates a new tree and recursively replaces parameter names
 * in the current node and all its children according to the replacement map.
 * It's commonly used in quantified expressions (exists, forall) to substitute
 * parameters with specific values.
 *
 * @param[in] tree The PDDL expression tree.
 * @param[in] node_id The identifier of the node to start replacement.
 * @param[in] replace Map of original parameter names to replacement names.
 * @return plansys2_msgs::msg::Tree The modified tree with replaced parameter names.
 */
plansys2_msgs::msg::Tree replace_children_param(
  const plansys2_msgs::msg::Tree & tree,
  const uint8_t & node_id,
  const std::map<std::string, std::string> & replace);

/**
 * @brief Compute the Cartesian product of vectors of strings.
 * This function recursively computes all possible combinations of elements
 * from multiple vectors. For example, given vectors [a,b] and [1,2],
 * it generates combinations [a,1], [a,2], [b,1], [b,2].
 * It's used primarily for expanding quantified expressions in PDDL.
 *
 * @param[out] rvvi Result vector containing all combinations.
 * @param[out] rvi Current combination being built.
 * @param[in] me Iterator to the current vector in the input.
 * @param[in] end Iterator to the end of the input vectors.
 */
void cart_product(
  std::vector<std::vector<std::string>> & rvvi,
  std::vector<std::string> & rvi,
  std::vector<std::vector<std::string>>::const_iterator me,
  std::vector<std::vector<std::string>>::const_iterator end);
}  // namespace plansys2


#endif  // PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
