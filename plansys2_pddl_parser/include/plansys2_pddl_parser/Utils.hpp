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

#ifndef PLANSYS2_PDDL_PARSER__UTILS_HPP_
#define PLANSYS2_PDDL_PARSER__UTILS_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/metric.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace parser
{
namespace pddl
{

/// Deepest parenthesis nesting accepted in PDDL text: deeper input is rejected
/// instead of risking a stack overflow in the recursive parsers
constexpr int kMaxNestingDepth = 256;

/**
 * \brief Returns the deepest parenthesis nesting in a PDDL text, ignoring comments.
 * \param[in] expr The PDDL text
 * \return The nesting depth, or -1 if the parentheses are not balanced
 */
int getMaxNesting(const std::string & expr);

/**
 * \brief Checks that a tree can be walked safely: child ids in range, every node
 *        reached once from the root (no cycles or shared nodes), the number of
 *        children each node type needs, and no deeper than kMaxNestingDepth.
 * \param[in] tree The tree to check; an empty tree is valid
 * \param[out] error Why the tree is not valid
 * \return true if the tree is valid
 */
bool validateTree(const plansys2_msgs::msg::Tree & tree, std::string & error);

/**
 * \brief Checks that a string is a PDDL name: a letter followed by letters, digits,
 *        '-' or '_'.
 * \param[in] name The candidate name
 * \return true if it can be used as a PDDL name
 */
bool isValidName(const std::string & name);

/**
 * \brief Reads a whole text as a PDDL number, whatever the locale: an optional sign,
 *        digits with an optional point, and an optional exponent ("1e-7").
 * \param[in] text The text; blanks around it are ignored
 * \param[out] value The number, only set when the text is one
 * \return true if the text is a finite number
 */
bool parseNumber(const std::string & text, double & value);

/**
 * \brief Leaves one space between tokens, none after '(' or before ')' and none at the
 *        ends; tabs and newlines count as spaces.
 * \param[in] expr The expression to be reduced
 * \return The reduced expression
 */
std::string getReducedString(const std::string & expr);

/// Returns the node type of an expression, given by its operator (the first token
/// after its '('), never by a name that contains one
/**
 * \param[in] expr The expression
 * \param[in] def The type when there is no operator, e.g. PREDICATE or FUNCTION
 * \return The node type
 */
uint8_t getNodeType(const std::string & expr, uint8_t def = plansys2_msgs::msg::Node::UNKNOWN);

/// Returns expression type and start position of an expression in a string
/**
 * \param[in] input The input string
 * \return result <- tuple(uint8_t, int)
 *         result(0) The expression type
 *         result(1) The position of the operator, INT_MAX if there is none
 */
std::tuple<uint8_t, int> getExpr(const std::string & input);

/// Returns expression type corresponding to the string input
/**
 * \param[in] input The input string
 * \return The expression type
 */
uint8_t getExprType(const std::string & input);

/// Returns function modifier type and start position of a function modifier in a string
/**
 * \param[in] input The input string
 * \return result <- tuple(uint8_t, int)
 *         result(0) The function modifier type
 *         result(1) The position of the modifier, INT_MAX if there is none
 */
std::tuple<uint8_t, int> getFunMod(const std::string & input);

/// Returns function modifier type corresponding to the string input
/**
 * \param[in] input The input string
 * \return The function modifier type
 */
uint8_t getFunModType(const std::string & input);

int getParenthesis(const std::string & wexpr, int start);

void removeOperatorBeforeParenthesis(std::string & wexpr);

std::vector<std::string> getSubExpr(const std::string & expr);

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::Action> action);

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::DurativeAction> action);

std::string toString(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id = 0, bool negate = false);

std::string toString(const plansys2_msgs::msg::Node & node);

std::string toStringPredicate(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringFunction(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringNumber(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

/// @brief Writes a number for PDDL, in fixed notation and whatever the locale.
/// @param[in] value The number.
/// @param[in] decimals Decimals written when they are enough to read back the same value;
///   otherwise, as many digits as needed are written. Negative: always the shortest form.
/// @return The number as text, e.g. "200.000000" or "0.0000001" ("200" if shortest).
std::string numberToString(double value, int decimals = 6);

std::string toStringAnd(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringOr(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringNot(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringExpression(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringFunctionModifier(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringConstant(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringParameter(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringExists(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringForall(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringImply(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

// "(when condition effect)"
std::string toStringWhen(const plansys2_msgs::msg::Tree & tree, uint32_t node_id);

/// PDDL3 modal constraint: how many times and conditions it takes; {0, 0} if it is not one
std::pair<unsigned, unsigned> constraintArity(const std::string & op);

/// A preference or a modal constraint, its operator ("at end" in full) and its arguments;
/// false for anything else, such as a predicate called always
bool splitModal(const std::string & expr, std::string & op, std::vector<std::string> & args);

std::string toStringPreference(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringConstraint(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

/// ?duration as a number: the duration a plan gives the action
void bindDuration(plansys2_msgs::msg::Tree & tree, double duration);

/// #t as a number: the time a continuous effect acts in one step of its integration
void bindTime(plansys2_msgs::msg::Tree & tree, double time);

/// This function creates a complete tree.
/**
 * This function recursivelly extracts the logic expressions and predicates from the expression.
 *
 * \param[in] tree The tree object to be created
 * \param[in] expr A string containing predicates and logic operators
 * \param[in] construct A string containing the associated PDDL constructs
 * \return A smart pointer to the node created
*/
plansys2_msgs::msg::Node::SharedPtr fromString(
  plansys2_msgs::msg::Tree & tree, const std::string & expr, bool negate = false,
  uint8_t parent = plansys2_msgs::msg::Node::UNKNOWN);

plansys2_msgs::msg::Tree fromString(
  const std::string & expr, bool negate = false,
  uint8_t parent = plansys2_msgs::msg::Node::UNKNOWN);

plansys2_msgs::msg::Node fromStringPredicate(const std::string & predicate);

plansys2_msgs::msg::Node fromStringExists(const std::string & exists);

plansys2_msgs::msg::Node fromStringFunction(const std::string & function);

plansys2_msgs::msg::Param fromStringParam(const std::string & name, const std::string & type = {});

plansys2_msgs::msg::Tree fromPredicates(const std::vector<std::string> & preds);

plansys2_msgs::msg::Tree::SharedPtr fromSubtree(
  const plansys2_msgs::msg::Tree & subtree, uint8_t node_type);

plansys2_msgs::msg::Tree::SharedPtr fromSubtrees(
  const std::vector<plansys2_msgs::msg::Tree> & subtrees, uint8_t node_type);

std::vector<uint32_t> getSubtreeIds(const plansys2_msgs::msg::Tree & tree);

std::vector<plansys2_msgs::msg::Tree> getSubtrees(const plansys2_msgs::msg::Tree & tree);

void getSubtreeChildren(
  plansys2_msgs::msg::Tree & subtree, const plansys2_msgs::msg::Tree & tree, uint32_t tree_parent,
  uint32_t subtree_parent);

void getPredicates(
  std::vector<plansys2_msgs::msg::Node> & predicates, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id = 0, bool negate = false);

void getFunctions(
  std::vector<plansys2_msgs::msg::Node> & functions, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id = 0, bool negate = false);

bool checkTreeEquality(
  const plansys2_msgs::msg::Tree & first, const plansys2_msgs::msg::Tree & second);

bool checkNodeEquality(
  const plansys2_msgs::msg::Node & first,
  const plansys2_msgs::msg::Node & second,
  bool check_var_params = true);

bool checkActionEquality(
  const plansys2_msgs::msg::Action & first, const plansys2_msgs::msg::Action & second);

bool checkDurativeActionEquality(
  const plansys2_msgs::msg::DurativeAction & first,
  const plansys2_msgs::msg::DurativeAction & second);

// Returns true if types match and:
//   - In var-param mode: names must be exactly equal
//   - In non-var-param mode: if either names are variable-like, allow any match;
bool checkParamEquality(
  const plansys2_msgs::msg::Param & first,
  const plansys2_msgs::msg::Param & second,
  bool check_var_params = true);

bool empty(const plansys2_msgs::msg::Tree & tree);

// Check if type of second is equal or a subtype of first
bool checkParamTypeEquivalence(
  const plansys2_msgs::msg::Param & first, const plansys2_msgs::msg::Param & second);

bool compare_str_case_insensitive(const std::string & a, const std::string & b);

/// Parses a metric: "minimize <expr>" or "maximize <expr>", optionally as "(:metric ...)".
/**
 * \param[in] metric The metric as a string, e.g. "minimize (+ (cost) (* 10 (total-time)))"
 * \return The metric, with an empty tree if the string is not a valid metric
*/
plansys2_msgs::msg::Metric metricFromString(const std::string & metric);

/// The metric as "minimize <expr>" or "maximize <expr>"; empty if there is no metric.
std::string toString(const plansys2_msgs::msg::Metric & metric);


}  // namespace pddl
}  // namespace parser

#endif  // PLANSYS2_PDDL_PARSER__UTILS_HPP_
