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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2
{

std::tuple<bool, std::vector<std::map<std::string, std::string>>> unifyPredicate(
  const plansys2::Predicate & predicate,
  const std::unordered_set<plansys2::Predicate> & predicates);

std::tuple<bool, std::vector<std::map<std::string, std::string>>> unifyFunction(
  const plansys2::Function & function, const std::unordered_set<plansys2::Function> & functions);

void mergeParamsValuesDicts(
  const std::map<std::string, std::string> & dict1,
  const std::map<std::string, std::string> & dict2, std::map<std::string, std::string> & dict3);
std::vector<std::map<std::string, std::string>> mergeParamsValuesVector(
  const std::vector<std::map<std::string, std::string>> & vector1,
  const std::vector<std::map<std::string, std::string>> & vector2);
std::vector<std::map<std::string, std::string>> complementParamsValuesVector(
  const std::vector<plansys2_msgs::msg::Param> & params,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances);

std::tuple<bool, std::vector<std::map<std::string, std::string>>> negateResult(
  const plansys2_msgs::msg::Node & node, const bool & result,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances);

std::tuple<bool, std::vector<std::map<std::string, std::string>>> negateResult(
  const std::vector<plansys2_msgs::msg::Param> & params, const bool & result,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances);

std::vector<plansys2_msgs::msg::Param> get_node_children_free_parameters(
  const plansys2_msgs::msg::Tree & tree, const plansys2_msgs::msg::Node & current_node);
void get_node_children_free_parameters_impl(
  const plansys2_msgs::msg::Tree & tree, const plansys2_msgs::msg::Node & current_node,
  std::vector<plansys2_msgs::msg::Param> & params, std::unordered_set<std::string> & seen,
  std::unordered_set<std::string> & exists_params);

std::vector<plansys2_msgs::msg::Param> get_node_free_parameters(
  const plansys2_msgs::msg::Node & node);
void get_node_free_parameters_impl(
  const plansys2_msgs::msg::Node & node, std::vector<plansys2_msgs::msg::Param> & params,
  std::unordered_set<std::string> & seen);

void solveDerivedPredicates(
  plansys2::State & state, const std::vector<plansys2_msgs::msg::Node> & root_nodes);

void solveDerivedPredicates(plansys2::State & state);
bool evaluateSCC(
  const std::vector<Derived> & scc, plansys2::State & state,
  const std::vector<plansys2_msgs::msg::Node> & root_nodes,
  std::unordered_set<plansys2::Derived> & unground_cache);

void groundPredicate(
  plansys2::State & new_state, const plansys2::Derived & derived,
  const std::vector<std::map<std::string, std::string>> & params_values_vector);

/// Evaluate a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \param[in] instances Current instances state.
 * \param[in] predicates Current predicates state.
 * \param[in] functions Current functions state.
 * \param[in] apply Apply result to problem expert or state.
 * \param[in] use_state Use state representation or problem client.
 * \param[in] negate Invert the truth value.
 * \return result <- tuple(bool, bool, double)
 *         result(0) true if success
 *         result(1) truth value of boolean expression
 *         result(2) value of numeric expression
 *         result(3) vector with the set of possible  values for the expression parameters
 */
std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree, const plansys2::State & state, uint8_t node_id = 0,
  bool negate = false);

std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, uint32_t node_id = 0,
  bool negate = false);

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
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, uint32_t node_id = 0,
  bool negate = false);

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
  const plansys2_msgs::msg::Tree & tree, const plansys2::State & state, uint32_t node_id = 0,
  bool negate = false);

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
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, uint32_t node_id = 0,
  bool negate = false, bool derive = true);

bool apply(
  const plansys2_msgs::msg::Tree & tree, plansys2::State & state, uint32_t node_id = 0,
  bool negate = false, bool derive = true);

bool apply(
  const plansys2_msgs::msg::Tree & tree, plansys2::State & state,
  std::vector<plansys2_msgs::msg::Node> & nodes_modified, uint32_t node_id = 0, bool negate = false,
  bool derive = true);

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
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, plansys2::State & state,
  std::vector<plansys2_msgs::msg::Node> & nodes_modified, bool use_state = false,
  uint32_t node_id = 0, bool negate = false, bool derive = true);

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

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
