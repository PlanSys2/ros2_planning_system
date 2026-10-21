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

#include "plansys2_pddl_parser/Utils.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace parser
{
namespace pddl
{

static void printN(std::ostream & s, uint8_t node_type)
{
  switch (node_type) {
    case plansys2_msgs::msg::Node::UNKNOWN:
      s << "NODE TYPE UNKNOWN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::AND:
      s << "NODE TYPE AND" << std::endl;
      break;
    case plansys2_msgs::msg::Node::OR:
      s << "NODE TYPE OR" << std::endl;
      break;
    case plansys2_msgs::msg::Node::NOT:
      s << "NODE TYPE NOT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::PREDICATE:
      s << "NODE TYPE PRED" << std::endl;
      break;
    case plansys2_msgs::msg::Node::FUNCTION:
      s << "NODE TYPE FUNC" << std::endl;
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      s << "NODE TYPE EXPR" << std::endl;
      break;
    case plansys2_msgs::msg::Node::NUMBER:
      s << "NODE TYPE NUM" << std::endl;
      break;
    case plansys2_msgs::msg::Node::EXISTS:
      s << "NODE TYPE EXISTS" << std::endl;
      break;
    case plansys2_msgs::msg::Node::FORALL:
      s << "NODE TYPE FORALL" << std::endl;
      break;
    case plansys2_msgs::msg::Node::IMPLY:
      s << "NODE TYPE IMPLY" << std::endl;
      break;
    case plansys2_msgs::msg::Node::WHEN:
      s << "NODE TYPE WHEN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::PREFERENCE:
      s << "NODE TYPE PREFERENCE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::CONSTRAINT:
      s << "NODE TYPE CONSTRAINT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_GE:
      s << "NODE TYPE COMP GE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_GT:
      s << "NODE TYPE COMP GT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_LE:
      s << "NODE TYPE COMP LE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_LT:
      s << "NODE TYPE COMP LT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_EQ:
      s << "NODE TYPE COMP EQ" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_ADD:
      s << "NODE TYPE ARITH ADD" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_SUB:
      s << "NODE TYPE ARITH SUB" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_MULT:
      s << "NODE TYPE ARITH MULT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_DIV:
      s << "NODE TYPE ARITH DIV" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ASSIGN:
      s << "NODE TYPE ASSIGN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::INCREASE:
      s << "NODE TYPE INCREASE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::DECREASE:
      s << "NODE TYPE DECREASE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::SCALE_UP:
      s << "NODE TYPE SCALE UP" << std::endl;
      break;
    case plansys2_msgs::msg::Node::SCALE_DOWN:
      s << "NODE TYPE SCALE DOWN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      s << "NODE TYPE FUNCTION MODIFIER" << std::endl;
      break;
    default:
      s << "NODE TYPE UNDEFINED" << std::endl;
      break;
  }
}

int getMaxNesting(const std::string & expr)
{
  int depth = 0;
  int max_depth = 0;
  bool in_comment = false;
  for (char ch : expr) {
    if (in_comment) {
      in_comment = ch != '\n';
    } else if (ch == ';') {
      in_comment = true;
    } else if (ch == '(') {
      max_depth = std::max(max_depth, ++depth);
    } else if (ch == ')' && --depth < 0) {
      return -1;
    }
  }
  return depth == 0 ? max_depth : -1;
}

bool validateTree(const plansys2_msgs::msg::Tree & tree, std::string & error)
{
  if (tree.nodes.empty()) {
    return true;
  }

  std::vector<bool> visited(tree.nodes.size(), false);
  // Iterative walk: a malformed tree must not be able to overflow the stack here
  std::vector<std::pair<uint32_t, int>> pending = {{0, 1}};
  visited[0] = true;

  while (!pending.empty()) {
    auto [id, depth] = pending.back();
    pending.pop_back();
    const auto & node = tree.nodes[id];

    if (depth > kMaxNestingDepth) {
      error = "tree deeper than " + std::to_string(kMaxNestingDepth) + " levels";
      return false;
    }

    size_t min_children = 0;
    size_t max_children = 0;
    switch (node.node_type) {
      case plansys2_msgs::msg::Node::AND:
      case plansys2_msgs::msg::Node::OR:
        max_children = node.children.size();
        break;
      case plansys2_msgs::msg::Node::NOT:
        min_children = max_children = 1;
        break;
      case plansys2_msgs::msg::Node::EXISTS:
        min_children = 1;
        max_children = node.children.size();
        break;
      case plansys2_msgs::msg::Node::FORALL:
        min_children = max_children = 1;
        break;
      case plansys2_msgs::msg::Node::IMPLY:
      case plansys2_msgs::msg::Node::WHEN:
        min_children = max_children = 2;
        break;
      case plansys2_msgs::msg::Node::PREFERENCE:
        min_children = max_children = 1;
        break;
      case plansys2_msgs::msg::Node::CONSTRAINT: {
          const auto [times, conds] = constraintArity(node.name);
          if (conds == 0) {
            error = "node " + std::to_string(id) + " is no constraint: " + node.name;
            return false;
          }
          min_children = max_children = times + conds;
          break;
        }
      case plansys2_msgs::msg::Node::EXPRESSION:
        min_children = max_children = 2;
        break;
      case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
        // Trees built before (total-cost) was a child have only the value
        min_children = 1;
        max_children = 2;
        break;
      case plansys2_msgs::msg::Node::PREDICATE:
      case plansys2_msgs::msg::Node::FUNCTION:
      case plansys2_msgs::msg::Node::NUMBER:
      case plansys2_msgs::msg::Node::CONSTANT:
      case plansys2_msgs::msg::Node::PARAMETER:
        break;
      default:
        error = "node " + std::to_string(id) + " has unknown type " +
          std::to_string(node.node_type);
        return false;
    }
    if (node.children.size() < min_children || node.children.size() > max_children) {
      error = "node " + std::to_string(id) + " has " + std::to_string(node.children.size()) +
        " children";
      return false;
    }

    for (auto child : node.children) {
      if (child >= tree.nodes.size()) {
        error = "node " + std::to_string(id) + " has child " + std::to_string(child) +
          " out of range";
        return false;
      }
      if (visited[child]) {
        error = "node " + std::to_string(child) + " is reached twice (cycle or shared node)";
        return false;
      }
      visited[child] = true;
      pending.push_back({child, depth + 1});
    }
  }
  return true;
}

bool isValidName(const std::string & name)
{
  auto is_letter = [](char c) {return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');};
  if (name.empty() || !is_letter(name[0])) {
    return false;
  }
  return std::all_of(
    name.begin(), name.end(), [&](char c) {
      return is_letter(c) || (c >= '0' && c <= '9') || c == '-' || c == '_';
    });
}

namespace
{
bool isBlank(char c)
{
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
}

// The token that starts at pos: up to a blank or a parenthesis. A symbolic operator
// ends where its symbols end, as in "(=?a ?b)"; "-" or "+" before a digit is a sign
std::string tokenAt(const std::string & s, size_t pos)
{
  if (pos < s.size() && std::string("<>=*/+-").find(s[pos]) != std::string::npos) {
    const bool sign = (s[pos] == '-' || s[pos] == '+') && pos + 1 < s.size() &&
      ((s[pos + 1] >= '0' && s[pos + 1] <= '9') || s[pos + 1] == '.');
    if (!sign) {
      const bool two = (s[pos] == '<' || s[pos] == '>') && pos + 1 < s.size() && s[pos + 1] == '=';
      return s.substr(pos, two ? 2 : 1);
    }
  }
  size_t end = pos;
  while (end < s.size() && !isBlank(s[end]) && s[end] != '(' && s[end] != ')') {
    end++;
  }
  return s.substr(pos, end - pos);
}

// The operator of an expression: the first token after its '(' (and its position),
// or the first token itself when the text does not start with '('
std::pair<std::string, int> headToken(const std::string & s)
{
  size_t pos = 0;
  while (pos < s.size() && isBlank(s[pos])) {
    pos++;
  }
  if (pos < s.size() && s[pos] == '(') {
    pos++;
    while (pos < s.size() && isBlank(s[pos])) {
      pos++;
    }
  }
  return {tokenAt(s, pos), static_cast<int>(pos)};
}

constexpr int kNotFound = std::numeric_limits<int>::max();

uint8_t expressionType(const std::string & op)
{
  if (op == ">=") {return plansys2_msgs::msg::Node::COMP_GE;}
  if (op == ">") {return plansys2_msgs::msg::Node::COMP_GT;}
  if (op == "<=") {return plansys2_msgs::msg::Node::COMP_LE;}
  if (op == "<") {return plansys2_msgs::msg::Node::COMP_LT;}
  if (op == "=") {return plansys2_msgs::msg::Node::COMP_EQ;}
  if (op == "*") {return plansys2_msgs::msg::Node::ARITH_MULT;}
  if (op == "/") {return plansys2_msgs::msg::Node::ARITH_DIV;}
  if (op == "+") {return plansys2_msgs::msg::Node::ARITH_ADD;}
  if (op == "-") {return plansys2_msgs::msg::Node::ARITH_SUB;}
  return plansys2_msgs::msg::Node::UNKNOWN;
}

uint8_t modifierType(const std::string & op)
{
  if (op == "assign") {return plansys2_msgs::msg::Node::ASSIGN;}
  if (op == "increase") {return plansys2_msgs::msg::Node::INCREASE;}
  if (op == "decrease") {return plansys2_msgs::msg::Node::DECREASE;}
  if (op == "scale-up") {return plansys2_msgs::msg::Node::SCALE_UP;}
  if (op == "scale-down") {return plansys2_msgs::msg::Node::SCALE_DOWN;}
  return plansys2_msgs::msg::Node::UNKNOWN;
}

uint8_t logicType(const std::string & op)
{
  if (op == "and") {return plansys2_msgs::msg::Node::AND;}
  if (op == "or") {return plansys2_msgs::msg::Node::OR;}
  if (op == "not") {return plansys2_msgs::msg::Node::NOT;}
  if (op == "exists") {return plansys2_msgs::msg::Node::EXISTS;}
  if (op == "forall") {return plansys2_msgs::msg::Node::FORALL;}
  if (op == "imply") {return plansys2_msgs::msg::Node::IMPLY;}
  if (op == "when") {return plansys2_msgs::msg::Node::WHEN;}
  return plansys2_msgs::msg::Node::UNKNOWN;
}
}  // namespace

bool parseNumber(const std::string & text, double & value)
{
  size_t begin = 0;
  size_t end = text.size();
  while (begin < end && isBlank(text[begin])) {
    begin++;
  }
  while (end > begin && isBlank(text[end - 1])) {
    end--;
  }
  if (begin < end && text[begin] == '+') {  // from_chars does not take '+'
    begin++;
    if (begin < end && text[begin] == '-') {
      return false;
    }
  }
  if (begin == end) {
    return false;
  }
  // Digits, a point or a sign first: no "inf" or "nan" in PDDL
  const char c = text[begin];
  if (!((c >= '0' && c <= '9') || c == '.' || c == '-')) {
    return false;
  }
  double parsed = 0.0;
  const char * first = text.data() + begin;
  const char * last = text.data() + end;
  auto [ptr, ec] = std::from_chars(first, last, parsed, std::chars_format::general);
  if (ec != std::errc() || ptr != last || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

std::string getReducedString(const std::string & expr)
{
  // One space between tokens, none after '(' or before ')', none at the ends
  std::string ret;
  ret.reserve(expr.size());
  bool blank = false;
  for (char c : expr) {
    if (isBlank(c)) {
      blank = true;
      continue;
    }
    if (blank && !ret.empty() && ret.back() != '(' && c != ')') {
      ret += ' ';
    }
    blank = false;
    ret += c;
  }
  return ret;
}

uint8_t getNodeType(const std::string & expr, uint8_t default_node_type)
{
  size_t start = 0;
  while (start < expr.size() && isBlank(expr[start])) {
    start++;
  }
  if (start == expr.size()) {
    return default_node_type;
  }

  if (expr[start] != '(') {
    const auto token = tokenAt(expr, start);
    double value = 0.0;
    // A variable, and #t: the time a continuous effect has acted
    if (!token.empty() && (token[0] == '?' || token == "#t")) {
      return plansys2_msgs::msg::Node::PARAMETER;
    }
    if (parseNumber(token, value)) {
      return plansys2_msgs::msg::Node::NUMBER;
    }
    return default_node_type;
  }

  // The type is given by the operator, never by a name that contains it
  const auto head = headToken(expr).first;
  if (head.empty()) {
    return default_node_type;
  }
  std::string op;
  std::vector<std::string> args;
  if (splitModal(expr, op, args)) {
    return op == "preference" ?
           plansys2_msgs::msg::Node::PREFERENCE : plansys2_msgs::msg::Node::CONSTRAINT;
  }
  if (head[0] == '?') {
    return plansys2_msgs::msg::Node::PARAMETER;
  }
  if (logicType(head) != plansys2_msgs::msg::Node::UNKNOWN) {
    return logicType(head);
  }
  if (modifierType(head) != plansys2_msgs::msg::Node::UNKNOWN) {
    return plansys2_msgs::msg::Node::FUNCTION_MODIFIER;
  }
  if (expressionType(head) != plansys2_msgs::msg::Node::UNKNOWN) {
    return plansys2_msgs::msg::Node::EXPRESSION;
  }
  return default_node_type;
}

std::tuple<uint8_t, int> getExpr(const std::string & input)
{
  const auto [head, position] = headToken(input);
  const auto type = expressionType(head);
  return std::make_tuple(type, type == plansys2_msgs::msg::Node::UNKNOWN ? kNotFound : position);
}

uint8_t getExprType(const std::string & input)
{
  return std::get<0>(getExpr(input));
}

std::tuple<uint8_t, int> getFunMod(const std::string & input)
{
  const auto [head, position] = headToken(input);
  const auto type = modifierType(head);
  return std::make_tuple(type, type == plansys2_msgs::msg::Node::UNKNOWN ? kNotFound : position);
}

uint8_t getFunModType(const std::string & input)
{
  return std::get<0>(getFunMod(input));
}

int getParenthesis(const std::string & wexpr, int start)
{
  int it = start + 1;
  int balance = 1;

  while (it < static_cast<int>(wexpr.size())) {
    if (wexpr[it] == '(') {
      balance++;
    }
    if (wexpr[it] == ')') {
      balance--;
    }

    if (balance == 0) {
      return it;
    }

    it++;
  }

  return it;
}

void removeOperatorBeforeParenthesis(std::string & wexpr)
{
  // Only the operator that starts the text: a name that contains one is left alone
  size_t start = 0;
  while (start < wexpr.size() && isBlank(wexpr[start])) {
    start++;
  }
  const auto op = tokenAt(wexpr, start);
  size_t end = start + op.size();

  if (op == "exists" || op == "forall") {
    // With its variable list
    const size_t open = wexpr.find('(', end);
    const size_t close = open == std::string::npos ? open : wexpr.find(')', open);
    if (close == std::string::npos) {
      throw std::runtime_error("Malformed '" + op + "' expression: missing variable list");
    }
    end = close + 1;
  } else {
    const bool is_operator = logicType(op) != plansys2_msgs::msg::Node::UNKNOWN ||
      modifierType(op) != plansys2_msgs::msg::Node::UNKNOWN ||
      expressionType(op) != plansys2_msgs::msg::Node::UNKNOWN;
    if (!is_operator) {
      return;
    }
  }

  wexpr.erase(0, end);
  size_t first = 0;
  while (first < wexpr.size() && isBlank(wexpr[first])) {
    first++;
  }
  wexpr.erase(0, first);
}

std::vector<std::string> getSubExpr(const std::string & expr)
{
  std::vector<std::string> ret;

  std::string wexpr(expr);

  // Remove first ( and last )
  while (!wexpr.empty() && isBlank(wexpr.back())) {
    wexpr.pop_back();
  }
  while (!wexpr.empty() && isBlank(wexpr.front())) {
    wexpr.erase(0, 1);
  }
  if (wexpr.size() < 2 || wexpr.front() != '(' || wexpr.back() != ')') {
    return ret;
  }
  wexpr.pop_back();
  wexpr.erase(0, 1);

  removeOperatorBeforeParenthesis(wexpr);

  // Parse the inner content
  while (!wexpr.empty()) {
    std::string::size_type first = wexpr.find("(");

    // If there's a parenthesized subexpression
    if (first != std::string::npos) {
      if (first > 0) {
        // Extract everything before the parenthesis as a token
        std::string before = wexpr.substr(0, first);
        std::istringstream iss(before);
        std::string token;
        while (iss >> token) {
          ret.push_back(token);
        }
      }

      // Extract the parenthesized subexpression
      int last = getParenthesis(wexpr, static_cast<int>(first));
      ret.push_back(wexpr.substr(first, static_cast<std::string::size_type>(last) - first + 1));
      wexpr.erase(0, static_cast<std::string::size_type>(last) + 1);
    } else {
      // No more parentheses, split the remaining string by whitespace
      std::istringstream iss(wexpr);
      std::string token;
      while (iss >> token) {
        ret.push_back(token);
      }
      break;
    }
  }

  return ret;
}

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::Action> action)
{
  std::string ret = action->name;
  for (const auto & param : action->parameters) {
    ret = ret + " " + param.name;
  }
  return ret;
}

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::DurativeAction> action)
{
  std::string ret = action->name;
  for (const auto & param : action->parameters) {
    ret = ret + " " + param.name;
  }
  return ret;
}

std::string toString(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  std::string ret;
  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::PREDICATE:
      ret = toStringPredicate(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::FUNCTION:
      ret = toStringFunction(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::NUMBER:
      ret = toStringNumber(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::AND:
      ret = toStringAnd(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::OR:
      ret = toStringOr(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::NOT:
      ret = toStringNot(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      ret = toStringExpression(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      ret = toStringFunctionModifier(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::CONSTANT:
      ret = toStringConstant(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::PARAMETER:
      ret = toStringParameter(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::EXISTS:
      ret = toStringExists(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::FORALL:
      ret = toStringForall(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::IMPLY:
      ret = toStringImply(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::WHEN:
      ret = toStringWhen(tree, node_id);
      break;
    case plansys2_msgs::msg::Node::PREFERENCE:
      ret = toStringPreference(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::CONSTRAINT:
      ret = toStringConstraint(tree, node_id, negate);
      break;
    default:
      std::cerr << "Unsupported node to string conversion" << std::endl;
      break;
  }

  return ret;
}

std::string toString(const plansys2_msgs::msg::Node & node)
{
  if (
    node.node_type != plansys2_msgs::msg::Node::PREDICATE &&
    node.node_type != plansys2_msgs::msg::Node::FUNCTION)
  {
    std::cerr << "Unsupported node to string conversion" << std::endl;
    return {};
  }

  plansys2_msgs::msg::Tree tree;
  tree.nodes.push_back(node);

  return toString(tree);
}

std::string toStringPredicate(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  std::string ret;

  if (negate) {
    ret = "(not (" + tree.nodes[node_id].name;
  } else {
    ret = "(" + tree.nodes[node_id].name;
  }

  for (const auto & param : tree.nodes[node_id].parameters) {
    ret += " " + param.name;
  }

  if (negate) {
    ret += "))";
  } else {
    ret += ")";
  }

  return ret;
}

std::string toStringFunction(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  (void)negate;
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  std::string ret;

  ret = "(" + tree.nodes[node_id].name;
  for (const auto & param : tree.nodes[node_id].parameters) {
    ret += " " + param.name;
  }

  ret += ")";

  return ret;
}

std::string toStringNumber(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  (void)negate;
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  return numberToString(tree.nodes[node_id].value);
}

std::string numberToString(double value, int decimals)
{
  if (!std::isfinite(value)) {
    return std::to_string(value);  // Not representable in PDDL anyway
  }

  // Fixed notation needs up to ~310 digits before and ~330 after the point
  std::array<char, 1024> buffer {};
  char * const first = buffer.data();
  char * const last = first + buffer.size();
  if (decimals >= 0) {
    auto written = std::to_chars(first, last, value, std::chars_format::fixed, decimals);
    double read_back = 0.0;
    std::from_chars(first, written.ptr, read_back);
    if (read_back == value) {
      return std::string(first, written.ptr);
    }
  }
  // The shortest fixed form that reads back the same value
  auto written = std::to_chars(first, last, value, std::chars_format::fixed);
  return std::string(first, written.ptr);
}

std::string toStringAnd(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }

  std::string ret;

  if (!negate) {
    ret = "(and ";
  } else {
    ret = "(or ";
  }

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, negate);
  }
  ret += ")";

  return ret;
}

std::string toStringOr(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }

  std::string ret;

  if (!negate) {
    ret = "(or ";
  } else {
    ret = "(and ";
  }

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, negate);
  }
  ret += ")";

  return ret;
}

std::string toStringNot(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }
  return toString(tree, tree.nodes[node_id].children[0], !negate);
}

std::string toStringExpression(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.size() < 2) {
    return {};
  }

  std::string op;
  bool comparison = true;
  switch (tree.nodes[node_id].expression_type) {
    case plansys2_msgs::msg::Node::COMP_GE:
      op = ">=";
      break;
    case plansys2_msgs::msg::Node::COMP_GT:
      op = ">";
      break;
    case plansys2_msgs::msg::Node::COMP_LE:
      op = "<=";
      break;
    case plansys2_msgs::msg::Node::COMP_LT:
      op = "<";
      break;
    case plansys2_msgs::msg::Node::COMP_EQ:
      op = "=";
      break;
    case plansys2_msgs::msg::Node::ARITH_MULT:
      op = "*";
      comparison = false;
      break;
    case plansys2_msgs::msg::Node::ARITH_DIV:
      op = "/";
      comparison = false;
      break;
    case plansys2_msgs::msg::Node::ARITH_ADD:
      op = "+";
      comparison = false;
      break;
    case plansys2_msgs::msg::Node::ARITH_SUB:
      op = "-";
      comparison = false;
      break;
    default:
      std::cerr << "toStringExpression: Unknown expression type: ";
      printN(std::cerr, tree.nodes[node_id].expression_type);
      return {};
  }

  // Only a comparison can be negated; its operands never are
  const bool negated = negate && comparison;
  std::string ret = negated ? "(not (" + op : "(" + op;
  for (auto child_id : tree.nodes[node_id].children) {
    ret += " " + toString(tree, child_id, false);
  }
  ret += negated ? "))" : ")";

  return ret;
}

std::string toStringFunctionModifier(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  (void)negate;  // An effect, never negated
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  const auto & children = tree.nodes[node_id].children;
  if (children.empty() || children.size() > 2) {
    return {};
  }

  std::string ret;

  switch (tree.nodes[node_id].modifier_type) {
    case plansys2_msgs::msg::Node::ASSIGN:
      ret = "(assign";
      break;
    case plansys2_msgs::msg::Node::INCREASE:
      ret = "(increase";
      break;
    case plansys2_msgs::msg::Node::DECREASE:
      ret = "(decrease";
      break;
    case plansys2_msgs::msg::Node::SCALE_UP:
      ret = "(scale-up";
      break;
    case plansys2_msgs::msg::Node::SCALE_DOWN:
      ret = "(scale-down";
      break;
    default:
      std::cerr << "toStringFunctionModifier: Unknown modifier type: ";
      printN(std::cerr, tree.nodes[node_id].modifier_type);
      return {};
  }

  if (children.size() == 1) {  // Only the value: the function is (total-cost)
    ret += " (total-cost)";
  }
  for (auto child_id : children) {
    ret += " " + toString(tree, child_id, false);
  }
  ret += ")";

  return ret;
}

std::string toStringConstant(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  (void)negate;
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  return tree.nodes[node_id].name;
}

std::string toStringParameter(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  (void)negate;
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  return tree.nodes[node_id].parameters[0].name;
}

namespace
{
// (exists (?a ?b - t) ...) or (forall ...); the variables keep their types when known
std::string toStringQuantifier(
  const std::string & quantifier, const plansys2_msgs::msg::Tree & tree, uint32_t node_id,
  bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }

  std::string ret;
  if (negate) {
    ret = "(not ";
  }
  ret += "(" + quantifier + " (";

  bool first_param = true;
  for (const auto & param : tree.nodes[node_id].parameters) {
    ret += first_param ? param.name : " " + param.name;
    // object, the type of untyped variables, is left out: the text stays as written
    if (!param.type.empty() && param.type != "object") {
      ret += " - " + param.type;
    }
    first_param = false;
  }
  ret += ") ";

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, false);
  }
  ret += ")";

  if (negate) {
    ret += ")";
  }

  return ret;
}
}  // namespace

std::string toStringExists(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  return toStringQuantifier("exists", tree, node_id, negate);
}

std::string toStringForall(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  return toStringQuantifier("forall", tree, node_id, negate);
}

std::string toStringImply(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size() || tree.nodes[node_id].children.size() != 2) {
    return {};
  }

  std::string ret = negate ? "(not (imply " : "(imply ";
  ret += toString(tree, tree.nodes[node_id].children[0], false) + " ";
  ret += toString(tree, tree.nodes[node_id].children[1], false) + ")";
  if (negate) {
    ret += ")";
  }
  return ret;
}

void bindDuration(plansys2_msgs::msg::Tree & tree, double duration)
{
  for (auto & node : tree.nodes) {
    if (node.node_type == plansys2_msgs::msg::Node::PARAMETER &&
      (node.name == "?duration" ||
      (!node.parameters.empty() && node.parameters[0].name == "?duration")))
    {
      node.node_type = plansys2_msgs::msg::Node::NUMBER;
      node.name.clear();
      node.parameters.clear();
      node.value = duration;
    }
  }
}

void bindTime(plansys2_msgs::msg::Tree & tree, double time)
{
  for (auto & node : tree.nodes) {
    if (node.node_type == plansys2_msgs::msg::Node::PARAMETER &&
      (node.name == "#t" || (!node.parameters.empty() && node.parameters[0].name == "#t")))
    {
      node.node_type = plansys2_msgs::msg::Node::NUMBER;
      node.name.clear();
      node.parameters.clear();
      node.value = time;
    }
  }
}

std::pair<unsigned, unsigned> constraintArity(const std::string & op)
{
  if (op == "always" || op == "sometime" || op == "at-most-once" || op == "at end") {
    return {0, 1};
  }
  if (op == "within" || op == "hold-after") {return {1, 1};}
  if (op == "sometime-after" || op == "sometime-before") {return {0, 2};}
  if (op == "always-within") {return {1, 2};}
  if (op == "hold-during") {return {2, 1};}
  return {0, 0};
}

bool splitModal(const std::string & expr, std::string & op, std::vector<std::string> & args)
{
  auto [head, pos] = headToken(expr);
  if (head != "preference" && head != "at" && constraintArity(head).second == 0) {
    return false;
  }
  // The arguments after the operator: names and numbers, and texts in parentheses
  std::string rest = getReducedString(expr);
  if (rest.size() < 2 || rest.front() != '(' || rest.back() != ')') {
    return false;
  }
  rest = rest.substr(1, rest.size() - 2);
  rest.erase(0, rest.find(head) + head.size());
  args.clear();
  size_t i = 0;
  while (i < rest.size()) {
    if (isBlank(rest[i])) {
      i++;
    } else if (rest[i] == '(') {
      const int last = getParenthesis(rest, static_cast<int>(i));
      if (last >= static_cast<int>(rest.size())) {
        return false;  // Unbalanced
      }
      args.push_back(rest.substr(i, static_cast<size_t>(last) - i + 1));
      i = static_cast<size_t>(last) + 1;
    } else {
      const auto token = tokenAt(rest, i);
      if (token.empty()) {
        return false;
      }
      args.push_back(token);
      i += token.size();
    }
  }
  auto is_cond = [](const std::string & a) {return !a.empty() && a.front() == '(';};
  double value = 0.0;
  if (head == "preference") {
    // (preference [name] C)
    if (args.empty() || args.size() > 2 || !is_cond(args.back()) ||
      (args.size() == 2 && is_cond(args[0])))
    {
      return false;
    }
    op = head;
    return true;
  }
  if (head == "at") {
    if (args.size() != 2 || args[0] != "end" || !is_cond(args[1])) {
      return false;
    }
    args.erase(args.begin());
    op = "at end";
    return true;
  }
  const auto [times, conds] = constraintArity(head);
  if (args.size() != times + conds) {
    return false;
  }
  for (size_t a = 0; a < args.size(); a++) {
    if (a < times ? !parseNumber(args[a], value) : !is_cond(args[a])) {
      return false;
    }
  }
  op = head;
  return true;
}

std::string toStringPreference(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size() || tree.nodes[node_id].children.size() != 1) {
    return {};
  }
  const auto & node = tree.nodes[node_id];
  std::string ret = "(preference " + (node.name.empty() ? "" : node.name + " ") +
    toString(tree, node.children[0], false) + ")";
  return negate ? "(not " + ret + ")" : ret;
}

std::string toStringConstraint(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }
  const auto & node = tree.nodes[node_id];
  std::string ret = "(" + node.name;
  for (const auto child : node.children) {
    ret += " " + toString(tree, child, false);
  }
  ret += ")";
  return negate ? "(not " + ret + ")" : ret;
}

std::string toStringWhen(const plansys2_msgs::msg::Tree & tree, uint32_t node_id)
{
  if (node_id >= tree.nodes.size() || tree.nodes[node_id].children.size() != 2) {
    return {};
  }
  return "(when " + toString(tree, tree.nodes[node_id].children[0], false) + " " +
         toString(tree, tree.nodes[node_id].children[1], false) + ")";
}

plansys2_msgs::msg::Node::SharedPtr fromString(
  plansys2_msgs::msg::Tree & tree, const std::string & expr, bool negate, uint8_t parent)
{
  std::string wexpr = getReducedString(expr);

  if (wexpr == "(and)" || wexpr == "(and )") {
    return nullptr;
  }

  // At the root, as in a goal or a condition, a name is a predicate
  auto default_node_type = plansys2_msgs::msg::Node::PREDICATE;
  switch (parent) {
    case plansys2_msgs::msg::Node::AND:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
    case plansys2_msgs::msg::Node::OR:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
    case plansys2_msgs::msg::Node::NOT:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      default_node_type = plansys2_msgs::msg::Node::FUNCTION;
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      default_node_type = plansys2_msgs::msg::Node::FUNCTION;
      break;
    case plansys2_msgs::msg::Node::EXISTS:
    case plansys2_msgs::msg::Node::FORALL:
    case plansys2_msgs::msg::Node::IMPLY:
    case plansys2_msgs::msg::Node::WHEN:
    case plansys2_msgs::msg::Node::PREFERENCE:
    case plansys2_msgs::msg::Node::CONSTRAINT:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
  }

  auto node_type = getNodeType(wexpr, default_node_type);

  switch (node_type) {
    case plansys2_msgs::msg::Node::AND: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, negate, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }

        return node;
      }
    case plansys2_msgs::msg::Node::OR:
    case plansys2_msgs::msg::Node::IMPLY: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, negate, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }

        return node;
      }
    case plansys2_msgs::msg::Node::WHEN: {
        // Never negated: a condition and an effect, each as written
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        tree.nodes.push_back(*node);

        for (const auto & e : getSubExpr(wexpr)) {
          auto child = fromString(tree, e, false, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }
        return node;
      }
    case plansys2_msgs::msg::Node::PREFERENCE:
    case plansys2_msgs::msg::Node::CONSTRAINT: {
        // Its conditions as written; the times of a constraint are its first children
        std::string op;
        std::vector<std::string> args;
        splitModal(wexpr, op, args);
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        if (node_type == plansys2_msgs::msg::Node::PREFERENCE) {
          node->name = args.size() == 2 ? args[0] : "";
          args.erase(args.begin(), args.end() - 1);
        } else {
          node->name = op;
        }
        tree.nodes.push_back(*node);
        for (const auto & arg : args) {
          auto child = fromString(tree, arg, false, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }
        return node;
      }
    case plansys2_msgs::msg::Node::NOT: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        auto child = subexprs.empty() ? nullptr : fromString(tree, subexprs[0], !negate, node_type);
        if (!child) {
          // A (not) with nothing to negate is dropped as a whole
          tree.nodes.pop_back();
          return nullptr;
        }
        tree.nodes[node->node_id].children.push_back(child->node_id);

        return node;
      }
    case plansys2_msgs::msg::Node::PREDICATE: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>(fromStringPredicate(wexpr));
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        return node;
      }
    case plansys2_msgs::msg::Node::FUNCTION: {
        if (wexpr.empty() || wexpr.front() != '(') {
          // A bare name is an object, as in (= ?r r2d2): functions go in parentheses
          if (!isValidName(wexpr)) {
            throw std::runtime_error("Wrong name [" + wexpr + "]");
          }
          auto node = std::make_shared<plansys2_msgs::msg::Node>();
          node->node_type = plansys2_msgs::msg::Node::CONSTANT;
          node->node_id = tree.nodes.size();
          node->name = wexpr;
          node->negate = negate;
          tree.nodes.push_back(*node);
          return node;
        }
        auto node = std::make_shared<plansys2_msgs::msg::Node>(fromStringFunction(wexpr));
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        return node;
      }
    case plansys2_msgs::msg::Node::EXPRESSION: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->expression_type = getExprType(wexpr);
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, false, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }

        return node;
      }
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->modifier_type = getFunModType(wexpr);
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, false, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }

        return node;
      }
    case plansys2_msgs::msg::Node::NUMBER: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        if (!parseNumber(wexpr, node->value)) {
          throw std::runtime_error("Wrong number [" + wexpr + "]");
        }
        node->negate = negate;
        tree.nodes.push_back(*node);

        return node;
      }

    case plansys2_msgs::msg::Node::EXISTS:
    case plansys2_msgs::msg::Node::FORALL: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>(fromStringExists(wexpr));
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, negate, node_type);
          if (child) {
            tree.nodes[node->node_id].children.push_back(child->node_id);
          }
        }

        return node;
      }

    case plansys2_msgs::msg::Node::PARAMETER: {
        // (?x) is ?x
        std::string name = wexpr;
        while (name.size() > 2 && name.front() == '(' && name.back() == ')') {
          name = getReducedString(name.substr(1, name.size() - 2));
        }
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->name = name;
        node->negate = negate;
        node->parameters.push_back(fromStringParam(name));
        tree.nodes.push_back(*node);

        return node;
      }
    // LCOV_EXCL_START
    default:
      std::cerr << "fromString: Error parsing expresion [" << wexpr << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return nullptr;
}

plansys2_msgs::msg::Tree fromString(const std::string & expr, bool negate, uint8_t parent)
{
  plansys2_msgs::msg::Tree tree;

  const int nesting = getMaxNesting(expr);
  if (nesting < 0 || nesting > kMaxNestingDepth) {
    std::cerr << "fromString: " << (nesting < 0 ? "unbalanced parentheses" : "nesting too deep") <<
      " in [" << expr << "]" << std::endl;
    return tree;
  }

  try {
    fromString(tree, expr, negate, parent);
  } catch (const std::exception & e) {
    std::cerr << "fromString: error parsing [" << expr << "]: " << e.what() << std::endl;
    tree.nodes.clear();
  }
  return tree;
}

plansys2_msgs::msg::Node fromStringPredicate(const std::string & predicate)
{
  plansys2_msgs::msg::Node ret;
  ret.node_type = plansys2_msgs::msg::Node::PREDICATE;

  std::string predicate_(predicate);
  if (!predicate_.empty() && predicate_.front() == '(') {
    predicate_.erase(0, 1);
  }
  if (!predicate_.empty() && predicate_.back() == ')') {
    predicate_.pop_back();
  }
  std::vector<std::string> tokens;
  std::istringstream iss(predicate_);
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  if (tokens.empty()) {
    throw std::runtime_error("Empty predicate");
  }

  ret.name = tokens[0];

  for (size_t i = 1; i < tokens.size(); i++) {
    plansys2_msgs::msg::Param param;
    param.name = tokens[i];
    ret.parameters.push_back(param);
  }

  ret.value = 0;

  return ret;
}

plansys2_msgs::msg::Node fromStringExists(const std::string & exists)
{
  // Also for forall: the caller sets the node type. "- type" applies to the variables before it
  plansys2_msgs::msg::Node ret;
  ret.node_type = plansys2_msgs::msg::Node::EXISTS;
  ret.value = 0;

  // The variable list is the first '(' after the quantifier
  const auto [quantifier, position] = headToken(exists);
  size_t open = exists.find("(", static_cast<size_t>(position) + quantifier.size());
  size_t close = (open == std::string::npos) ? std::string::npos : exists.find(")", open);
  if (close == std::string::npos) {
    return ret;
  }

  std::stringstream list(exists.substr(open + 1, close - open - 1));
  std::vector<std::string> tokens;
  for (std::string token; list >> token; ) {
    tokens.push_back(token);
  }

  size_t untyped = 0;  // first variable still without a type
  for (size_t i = 0; i < tokens.size(); i++) {
    if (tokens[i] == "-" && i + 1 < tokens.size()) {
      for (size_t k = untyped; k < ret.parameters.size(); k++) {
        ret.parameters[k].type = tokens[i + 1];
      }
      untyped = ret.parameters.size();
      i++;
    } else {
      plansys2_msgs::msg::Param param;
      param.name = tokens[i];
      ret.parameters.push_back(param);
    }
  }

  return ret;
}

plansys2_msgs::msg::Node fromStringFunction(const std::string & function)
{
  // "(f a b)", "(f a b) 1.5" or "(= (f a b) 1.5)": the name, the parameters and the value. The
  // value of an object fluent is an object: "(= (pos r1) kitchen)"
  plansys2_msgs::msg::Node ret;
  ret.node_type = plansys2_msgs::msg::Node::FUNCTION;

  // The tokens, and whether each one is inside the parentheses of the function
  std::vector<std::pair<std::string, bool>> tokens;
  int depth = 0;
  int function_depth = -1;
  for (size_t pos = 0; pos < function.size(); ) {
    if (function[pos] == '(') {
      depth++;
      pos++;
      continue;
    }
    if (function[pos] == ')') {
      if (depth == function_depth) {
        function_depth = -2;  // Closed: what follows is the value
      }
      depth--;
      pos++;
      continue;
    }
    if (isBlank(function[pos])) {
      pos++;
      continue;
    }
    auto token = tokenAt(function, pos);
    pos += token.size();
    if (token == "=" && tokens.empty()) {
      continue;
    }
    if (function_depth == -1) {
      function_depth = depth;  // The name opens the function
    }
    tokens.emplace_back(token, function_depth >= 0);
  }

  size_t i = 0;
  if (i < tokens.size()) {
    ret.name = tokens[i++].first;
  }
  for (; i < tokens.size(); i++) {
    double value = 0.0;
    if (parseNumber(tokens[i].first, value)) {
      ret.value = value;
      break;
    }
    if (!tokens[i].second) {
      ret.object = tokens[i].first;  // After the function: its object
      break;
    }
    plansys2_msgs::msg::Param param;
    param.name = tokens[i].first;
    ret.parameters.push_back(param);
  }

  return ret;
}

plansys2_msgs::msg::Param fromStringParam(const std::string & name, const std::string & type)
{
  plansys2_msgs::msg::Param ret;
  ret.name = name;
  ret.type = type;
  return ret;
}

plansys2_msgs::msg::Tree fromPredicates(const std::vector<std::string> & preds)
{
  plansys2_msgs::msg::Node node;
  node.node_type = plansys2_msgs::msg::Node::AND;
  node.node_id = 0;
  node.negate = false;

  plansys2_msgs::msg::Tree tree;
  tree.nodes.push_back(node);

  for (const auto & pred : preds) {
    auto child = fromStringPredicate(pred);
    child.node_id = tree.nodes.size();
    child.negate = false;
    tree.nodes.push_back(child);
    tree.nodes[0].children.push_back(child.node_id);
  }

  return tree;
}

plansys2_msgs::msg::Tree::SharedPtr fromSubtree(
  const plansys2_msgs::msg::Tree & subtree, uint8_t node_type)
{
  std::vector<plansys2_msgs::msg::Tree> temp;
  temp.push_back(subtree);
  return fromSubtrees(temp, node_type);
}

plansys2_msgs::msg::Tree::SharedPtr fromSubtrees(
  const std::vector<plansys2_msgs::msg::Tree> & subtrees, uint8_t node_type)
{
  if (
    node_type != plansys2_msgs::msg::Node::AND && node_type != plansys2_msgs::msg::Node::OR &&
    node_type != plansys2_msgs::msg::Node::NOT)
  {
    std::cerr << "fromSubtrees: Unsupported root type." << std::endl;
    return nullptr;
  }

  if (node_type == plansys2_msgs::msg::Node::NOT && subtrees.size() > 1) {
    std::cerr << "fromSubtree: A NOT node can only operate on single subtree." << std::endl;
    return nullptr;
  }

  if (subtrees.empty()) {
    std::cerr << "fromSubtree: The subtrees vector is empty." << std::endl;
    return nullptr;
  }

  plansys2_msgs::msg::Node node;
  node.node_type = node_type;
  node.node_id = 0;
  node.negate = false;

  auto tree = std::make_shared<plansys2_msgs::msg::Tree>();
  tree->nodes.push_back(node);

  for (unsigned i = 0; i < subtrees.size(); ++i) {
    auto tree_size = tree->nodes.size();
    tree->nodes[0].children.push_back(tree_size);
    tree->nodes.insert(
      std::end(tree->nodes), std::begin(subtrees[i].nodes), std::end(subtrees[i].nodes));
    for (unsigned j = 0; j < subtrees[i].nodes.size(); ++j) {
      tree->nodes[tree_size + j].node_id += tree_size;
      for (unsigned k = 0; k < subtrees[i].nodes[j].children.size(); ++k) {
        tree->nodes[tree_size + j].children[k] += tree_size;
      }
    }
    if (node_type == plansys2_msgs::msg::Node::NOT) {
      tree->nodes[1].negate = true;
      break;
    }
  }

  return tree;
}

std::vector<uint32_t> getSubtreeIds(const plansys2_msgs::msg::Tree & tree)
{
  if (tree.nodes.empty()) {  // No expression
    return {};
  }

  switch (tree.nodes.front().node_type) {
    case plansys2_msgs::msg::Node::AND: {
        return tree.nodes.front().children;
      }
    case plansys2_msgs::msg::Node::EXISTS: {
        return tree.nodes.front().children;
      }
    default:
      std::cerr << "getSubtreeIds: Error parsing expresion [" << toString(tree) << "]" << std::endl;
  }

  return {};
}

std::vector<plansys2_msgs::msg::Tree> getSubtrees(const plansys2_msgs::msg::Tree & tree)
{
  std::vector<uint32_t> node_ids = parser::pddl::getSubtreeIds(tree);
  std::vector<plansys2_msgs::msg::Tree> subtrees;
  for (auto node_id : node_ids) {
    plansys2_msgs::msg::Tree subtree;
    subtree.nodes.push_back(tree.nodes[node_id]);
    subtree.nodes[0].node_id = 0;
    subtree.nodes[0].children.clear();
    getSubtreeChildren(subtree, tree, node_id, 0);
    subtrees.push_back(subtree);
  }
  return subtrees;
}

void getSubtreeChildren(
  plansys2_msgs::msg::Tree & subtree, const plansys2_msgs::msg::Tree & tree, uint32_t tree_parent,
  uint32_t subtree_parent)
{
  for (auto child_id : tree.nodes[tree_parent].children) {
    auto subtree_size = subtree.nodes.size();
    subtree.nodes[subtree_parent].children.push_back(subtree_size);
    subtree.nodes.push_back(tree.nodes[child_id]);
    subtree.nodes.back().node_id = subtree_size;
    subtree.nodes.back().children.clear();
    getSubtreeChildren(subtree, tree, child_id, subtree_size);
  }
}

void getPredicates(
  std::vector<plansys2_msgs::msg::Node> & predicates, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::FUNCTION:
    case plansys2_msgs::msg::Node::EXPRESSION:
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
    case plansys2_msgs::msg::Node::NUMBER:
      // These cases have no meaning
      break;
    case plansys2_msgs::msg::Node::AND:
      for (auto child_id : tree.nodes[node_id].children) {
        getPredicates(predicates, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::OR:
      for (auto child_id : tree.nodes[node_id].children) {
        getPredicates(predicates, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::NOT:
      getPredicates(predicates, tree, tree.nodes[node_id].children[0], !negate);
      break;
    case plansys2_msgs::msg::Node::ACTION:
      for (auto child_id : tree.nodes[node_id].children) {
        getPredicates(predicates, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::PREDICATE:
      plansys2_msgs::msg::Node pred = tree.nodes[node_id];
      if (
        std::find_if(
          predicates.begin(), predicates.end(),
          [&](const plansys2_msgs::msg::Node & p) {
            return checkNodeEquality(p, pred, true);
          }
        ) == predicates.end())
      {
        pred.negate = negate;
        predicates.push_back(pred);
      }
      break;
  }
}

void getFunctions(
  std::vector<plansys2_msgs::msg::Node> & functions, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND:
    case plansys2_msgs::msg::Node::OR:
    case plansys2_msgs::msg::Node::NOT:
    case plansys2_msgs::msg::Node::NUMBER:
      // These cases have no meaning
      break;
    case plansys2_msgs::msg::Node::ACTION:
      for (auto child_id : tree.nodes[node_id].children) {
        getFunctions(functions, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      for (auto child_id : tree.nodes[node_id].children) {
        getFunctions(functions, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      for (auto child_id : tree.nodes[node_id].children) {
        getFunctions(functions, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::FUNCTION:
      plansys2_msgs::msg::Node func = tree.nodes[node_id];
      if (
        std::find_if(
          functions.begin(), functions.end(),
          [&](const plansys2_msgs::msg::Node & f) {
            return checkNodeEquality(f, func, true);
          }
        ) == functions.end())
      {
        func.value = 0.0;
        functions.push_back(func);
      }
      break;
  }
}

bool checkTreeEquality(
  const plansys2_msgs::msg::Tree & first, const plansys2_msgs::msg::Tree & second)
{
  if (first.nodes.size() != second.nodes.size()) {
    return false;
  }

  for (unsigned i = 0; i < first.nodes.size(); i++) {
    if (!checkNodeEquality(first.nodes[i], second.nodes[i])) {
      return false;
    }
  }

  return true;
}

bool checkNodeEquality(
  const plansys2_msgs::msg::Node & first,
  const plansys2_msgs::msg::Node & second,
  bool check_var_params)
{
  if (first.node_type != second.node_type) {
    return false;
  }

  if (
    first.node_type == plansys2_msgs::msg::Node::ACTION ||
    first.node_type == plansys2_msgs::msg::Node::PREDICATE ||
    first.node_type == plansys2_msgs::msg::Node::FUNCTION)
  {
    // PDDL is case insensitive (#256)
    if (!compare_str_case_insensitive(first.name, second.name)) {
      return false;
    }
  }

  if (first.node_type == plansys2_msgs::msg::Node::EXPRESSION) {
    if (first.expression_type != second.expression_type) {
      return false;
    }
  }

  if (first.node_type == plansys2_msgs::msg::Node::FUNCTION_MODIFIER) {
    if (first.modifier_type != second.modifier_type) {
      return false;
    }
  }

  if (first.node_type == plansys2_msgs::msg::Node::NUMBER) {
    if (std::fabs(first.value - second.value) > 1e-9) {
      return false;
    }
  }

  if (first.children.size() != second.children.size()) {
    return false;
  }

  if (first.parameters.size() != second.parameters.size()) {
    return false;
  }

  for (unsigned i = 0; i < first.parameters.size(); i++) {
    if (!checkParamEquality(first.parameters[i], second.parameters[i], check_var_params)) {
      return false;
    }
  }

  return true;
}

bool checkParamEquality(
  const plansys2_msgs::msg::Param & first,
  const plansys2_msgs::msg::Param & second,
  bool check_var_params)
{
  if (!check_var_params &&
    (first.name.front() == '?' || second.name.front() == '?'))
  {
    return true;
  }
  return compare_str_case_insensitive(first.name, second.name);
}

bool checkActionEquality(
  const plansys2_msgs::msg::Action & first, const plansys2_msgs::msg::Action & second)
{
  if (first.name != second.name) {
    return false;
  }

  if (first.parameters.size() != second.parameters.size()) {
    return false;
  }

  for (unsigned i = 0; i < first.parameters.size(); i++) {
    if (!checkParamEquality(first.parameters[i], second.parameters[i])) {
      return false;
    }
  }

  return parser::pddl::checkTreeEquality(first.preconditions, second.preconditions) &&
         parser::pddl::checkTreeEquality(first.effects, second.effects);
}

bool checkDurativeActionEquality(
  const plansys2_msgs::msg::DurativeAction & first,
  const plansys2_msgs::msg::DurativeAction & second)
{
  if (first.name != second.name) {
    return false;
  }
  if (first.parameters.size() != second.parameters.size()) {
    return false;
  }

  for (unsigned i = 0; i < first.parameters.size(); i++) {
    if (!checkParamEquality(first.parameters[i], second.parameters[i])) {
      return false;
    }
  }

  return parser::pddl::checkTreeEquality(
    first.at_start_requirements, second.at_start_requirements) &&
         parser::pddl::checkTreeEquality(
    first.over_all_requirements, second.over_all_requirements) &&
         parser::pddl::checkTreeEquality(first.at_end_requirements, second.at_end_requirements) &&
         parser::pddl::checkTreeEquality(first.at_start_effects, second.at_start_effects) &&
         parser::pddl::checkTreeEquality(first.at_end_effects, second.at_end_effects);
}

bool empty(const plansys2_msgs::msg::Tree & tree)
{
  if (tree.nodes.empty()) {
    return true;
  }

  if (
    (tree.nodes[0].node_type == plansys2_msgs::msg::Node::AND ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::OR ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::NOT ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::EXPRESSION ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::FUNCTION_MODIFIER) &&
    tree.nodes[0].children.empty())
  {
    return true;
  }

  return false;
}

bool checkParamTypeEquivalence(
  const plansys2_msgs::msg::Param & first, const plansys2_msgs::msg::Param & second)
{
  return first.type == "" || compare_str_case_insensitive(first.type, "object") ||
         compare_str_case_insensitive(first.type, second.type) ||
         std::find(first.sub_types.begin(), first.sub_types.end(), second.type) !=
         first.sub_types.end();
}

bool compare_str_case_insensitive(const std::string & a, const std::string & b)
{
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(), b.end(),
           [](unsigned char ac, unsigned char bc) {
             return std::tolower(ac) == std::tolower(bc);
          });
}

plansys2_msgs::msg::Metric metricFromString(const std::string & metric)
{
  plansys2_msgs::msg::Metric ret;
  const char * blanks = " \t\r\n";

  std::string s = metric;
  auto trim = [&](std::string & str) {
      str.erase(0, str.find_first_not_of(blanks));
      str.erase(str.find_last_not_of(blanks) + 1);
    };
  trim(s);

  // (:metric minimize <expr>) -> minimize <expr>
  if (!s.empty() && s.front() == '(') {
    std::string inner = s.substr(1);
    trim(inner);
    if (inner.rfind(":metric", 0) == 0 && s.back() == ')') {
      s = inner.substr(7, inner.size() - 8);
      trim(s);
    }
  }

  const auto split = s.find_first_of(blanks);
  if (split == std::string::npos) {
    return ret;
  }
  const std::string optimization = s.substr(0, split);
  std::string expr = s.substr(split);
  trim(expr);

  if (optimization == "minimize") {
    ret.optimization = plansys2_msgs::msg::Metric::MINIMIZE;
  } else if (optimization == "maximize") {
    ret.optimization = plansys2_msgs::msg::Metric::MAXIMIZE;
  } else {
    return ret;
  }

  auto tree = fromString(expr, false, plansys2_msgs::msg::Node::EXPRESSION);
  std::string error;
  if (tree.nodes.empty() || !validateTree(tree, error)) {
    return ret;
  }
  // Only numbers, functions and arithmetic: a comparison or a predicate is not a metric
  for (const auto & node : tree.nodes) {
    switch (node.node_type) {
      case plansys2_msgs::msg::Node::FUNCTION:
      case plansys2_msgs::msg::Node::NUMBER:
        break;
      case plansys2_msgs::msg::Node::EXPRESSION:
        if (node.expression_type != plansys2_msgs::msg::Node::ARITH_ADD &&
          node.expression_type != plansys2_msgs::msg::Node::ARITH_SUB &&
          node.expression_type != plansys2_msgs::msg::Node::ARITH_MULT &&
          node.expression_type != plansys2_msgs::msg::Node::ARITH_DIV)
        {
          return ret;
        }
        break;
      default:
        return ret;
    }
  }
  ret.tree = tree;
  return ret;
}

std::string toString(const plansys2_msgs::msg::Metric & metric)
{
  if (metric.tree.nodes.empty()) {
    return {};
  }
  const std::string optimization =
    metric.optimization == plansys2_msgs::msg::Metric::MAXIMIZE ? "maximize" : "minimize";
  return optimization + " " + toString(metric.tree);
}

}  // namespace pddl
}  // namespace parser
