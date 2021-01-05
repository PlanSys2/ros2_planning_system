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

#include "plansys2_pddl_parser/Tree.h"

#include <regex>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

namespace parser
{
namespace pddl
{
namespace tree
{

bool operator==(const Param & op1, const Param & op2)
{
  return op1.type == op2.type && op1.name == op2.name;
}

bool operator==(const Predicate & op1, const Predicate & op2)
{
  return op1.name == op2.name && op1.parameters == op2.parameters;
}

bool operator==(const Function & op1, const Function & op2)
{
  return op1.name == op2.name && op1.parameters == op2.parameters;
}

std::string getReducedString(const std::string & expr)
{
  std::regex nts_chars("[\n\t]*", std::regex_constants::ECMAScript);
  std::string ret = std::regex_replace(expr, nts_chars, "");
  std::regex open_paren("\\( ", std::regex_constants::ECMAScript);
  ret = std::regex_replace(ret, open_paren, "(");
  std::regex close_paren(" \\)", std::regex_constants::ECMAScript);
  ret = std::regex_replace(ret, close_paren, ")");
  return ret;
}

NodeType getNodeType(const std::string & expr)
{
  NodeType node_type = UNKNOWN;

  std::smatch match;
  int first = std::numeric_limits<int>::max();

  if (std::regex_search(expr, match, std::regex("\\(and"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = AND;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(or"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = OR;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(not"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = NOT;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(predicate"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = PREDICATE;
    }
  }

  // NOTE: "function_modifier" must precede "function"
  if (std::regex_search(expr, match, std::regex("\\(function_modifier"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = FUNCTION_MODIFIER;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(function"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = FUNCTION;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(expression"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = EXPRESSION;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(number"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = NUMBER;
    }
  }

  return node_type;
}

ExprType getExprType(const std::string & input)
{
  ExprType expr_type = UNKNOWN_EXPR_TYPE;

  std::smatch match;
  int first = std::numeric_limits<int>::max();

  if (std::regex_search(input, match, std::regex(">="))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = COMP_GE;
    }
  }

  if (std::regex_search(input, match, std::regex(">"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = COMP_GT;
    }
  }

  if (std::regex_search(input, match, std::regex("<="))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = COMP_LE;
    }
  }

  if (std::regex_search(input, match, std::regex("<"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = COMP_LT;
    }
  }

  if (std::regex_search(input, match, std::regex("\\*"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = ARITH_MULT;
    }
  }

  if (std::regex_search(input, match, std::regex("\\/"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = ARITH_DIV;
    }
  }

  return expr_type;
}

FunModType getFunModType(const std::string & input)
{
  FunModType fun_mod_type = UNKNOWN_FUN_MOD_TYPE;

  std::smatch match;
  int first = std::numeric_limits<int>::max();

  if (std::regex_search(input, match, std::regex("assign"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = ASSIGN;
    }
  }

  if (std::regex_search(input, match, std::regex("increase"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = INCREASE;
    }
  }

  if (std::regex_search(input, match, std::regex("decrease"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = DECREASE;
    }
  }

  if (std::regex_search(input, match, std::regex("scale-up"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = SCALE_UP;
    }
  }

  if (std::regex_search(input, match, std::regex("scale-down"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = SCALE_DOWN;
    }
  }

  return fun_mod_type;
}

int getParenthesis(const std::string & wexpr, int start)
{
  int it = start + 1;
  int balance = 1;

  while (it < wexpr.size()) {
    if (wexpr[it] == '(') {balance++;}
    if (wexpr[it] == ')') {balance--;}

    if (balance == 0) {
      return it;
    }

    it++;
  }

  return it;
}

std::vector<std::string> getSubExpr(const std::string & expr)
{
  std::vector<std::string> ret;

  std::string wexpr(expr);

  // Remove first ( and last )
  while (wexpr.back() == ' ') {wexpr.pop_back();}
  wexpr.pop_back();
  while (wexpr.front() == ' ') {wexpr.erase(0, 1);}
  wexpr.erase(0, 1);

  while (wexpr.size() > 0) {
    int first = wexpr.find("(");

    std::smatch match;
    std::regex num_regexp("[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)");
    bool found_num = std::regex_search(wexpr, match, num_regexp);

    if (found_num && first != std::string::npos) {
      if (match.position() < first) {
        ret.push_back(wexpr.substr(match.position(), match.length()));
        wexpr.erase(wexpr.begin(), wexpr.begin() + match.position() + match.length());
      }
      else {
        int last = getParenthesis(wexpr, first);
        ret.push_back(wexpr.substr(first, last - first + 1));
        wexpr.erase(wexpr.begin(), wexpr.begin() + last + 1);
      }
    }
    else if (found_num) {
      ret.push_back(wexpr.substr(match.position(), match.length()));
      wexpr.erase(wexpr.begin(), wexpr.begin() + match.position() + match.length());
    }
    else if (first != std::string::npos) {
      int last = getParenthesis(wexpr, first);
      ret.push_back(wexpr.substr(first, last - first + 1));
      wexpr.erase(wexpr.begin(), wexpr.begin() + last + 1);
    }
    else {
      break;
    }
  }

  return ret;
}

std::string getPredicateName(const std::string & expr)
{
  std::string ret(expr);

  size_t start = 1, end = 0;
  end = ret.find(" ", start);
  ret = ret.substr(start, (end == std::string::npos) ? std::string::npos : end - start);

  return ret;
}

std::vector<parser::pddl::tree::Param> getPredicateParams(const std::string & expr)
{
  std::vector<parser::pddl::tree::Param> ret;

  std::string wstring(expr);

  size_t start = 0, end = 0;
  end = wstring.find(" ", start);
  wstring = wstring.substr(end + 1, wstring.size() - end - 2);

  start = 0;
  while (end != std::string::npos) {
    end = wstring.find(" ", start);

    parser::pddl::tree::Param param;
    param.name = wstring.substr(
      start,
      (end == std::string::npos) ? std::string::npos : end - start);

    ret.push_back(param);

    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

std::string getFunctionName(const std::string & expr)
{
  std::string ret(expr);

  size_t start = 1, end = 0;
  end = ret.find(" ", start);
  ret = ret.substr(start, (end == std::string::npos) ? std::string::npos : end - start);

  return ret;
}

std::vector<parser::pddl::tree::Param> getFunctionParams(const std::string & expr)
{
  std::vector<parser::pddl::tree::Param> ret;

  std::string wstring(expr);

  size_t start = 0, end = 0;
  end = wstring.find(" ", start);
  wstring = wstring.substr(end + 1, wstring.size() - end - 2);

  start = 0;
  while (end != std::string::npos) {
    end = wstring.find(" ", start);

    parser::pddl::tree::Param param;
    param.name = wstring.substr(
      start,
      (end == std::string::npos) ? std::string::npos : end - start);

    ret.push_back(param);

    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

std::shared_ptr<TreeNode> get_tree_node(const std::string & expr,
                                        const std::string & construct)
{
  std::string wexpr = getReducedString(expr);
  std::string wconstruct = getReducedString(construct);

  NodeType node_type = getNodeType(wconstruct);

  if (wexpr == "(and)") {
    return nullptr;
  }
  switch (node_type) {
    case AND: {
        std::shared_ptr<parser::pddl::tree::AndNode> pn_and = std::make_shared<parser::pddl::tree::AndNode>();

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        std::vector<std::string> construct_subexprs = getSubExpr(wconstruct);

        for (unsigned i = 0; i < subexprs.size(); ++i ) {
          pn_and->ops.push_back(get_tree_node(subexprs[i], construct_subexprs[i]));
        }

        return pn_and;
      }

    case OR: {
        std::shared_ptr<parser::pddl::tree::OrNode> pn_or = std::make_shared<parser::pddl::tree::OrNode>();

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        std::vector<std::string> construct_subexprs = getSubExpr(wconstruct);

        for (unsigned i = 0; i < subexprs.size(); ++i ) {
          pn_or->ops.push_back(get_tree_node(subexprs[i], construct_subexprs[i]));
        }

        return pn_or;
      }

    case NOT: {
        std::shared_ptr<parser::pddl::tree::NotNode> pn_not = std::make_shared<parser::pddl::tree::NotNode>();

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        std::vector<std::string> construct_subexprs = getSubExpr(wconstruct);

        pn_not->op = get_tree_node(subexprs[0], construct_subexprs[0]);

        return pn_not;
      }

    case PREDICATE: {
        std::shared_ptr<parser::pddl::tree::PredicateNode> pred =
          std::make_shared<parser::pddl::tree::PredicateNode>();

        pred->predicate_.name = getPredicateName(wexpr);
        pred->predicate_.parameters = getPredicateParams(wexpr);

        return pred;
      }

    case FUNCTION: {
        std::shared_ptr<parser::pddl::tree::FunctionNode> func =
          std::make_shared<parser::pddl::tree::FunctionNode>();

        func->function_.name = getFunctionName(wexpr);
        func->function_.parameters = getFunctionParams(wexpr);

        return func;
    }

    case EXPRESSION: {
        std::shared_ptr<parser::pddl::tree::ExpressionNode> expression =
          std::make_shared<parser::pddl::tree::ExpressionNode>();

        expression->expr_type = getExprType(wexpr);

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        std::vector<std::string> construct_subexprs = getSubExpr(wconstruct);

        for (unsigned i = 0; i < subexprs.size(); ++i ) {
          expression->ops.push_back(get_tree_node(subexprs[i], construct_subexprs[i]));
        }

        return expression;
    }

    case FUNCTION_MODIFIER: {
        std::shared_ptr<parser::pddl::tree::FunctionModifierNode> fun_mod =
          std::make_shared<parser::pddl::tree::FunctionModifierNode>();

        fun_mod->modifier_type = getFunModType(wexpr);

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        std::vector<std::string> construct_subexprs = getSubExpr(wconstruct);

        for (unsigned i = 0; i < subexprs.size(); ++i ) {
          fun_mod->ops.push_back(get_tree_node(subexprs[i], construct_subexprs[i]));
        }

        return fun_mod;
    }

    case NUMBER: {
        std::shared_ptr<parser::pddl::tree::NumberNode> number_node =
          std::make_shared<parser::pddl::tree::NumberNode>();

        number_node->value_ = std::stod(wexpr);

        return number_node;
    }

    // LCOV_EXCL_START
    default:
      std::cerr << "get_tree_node: Error parsing expresion [" << wexpr << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return nullptr;
}

}  // namespace tree
}  // namespace pddl
}  // namespace parser
