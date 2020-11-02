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

#include "plansys2_domain_expert/Types.hpp"

#include <regex>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
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

NodeType getType(const std::string & expr)
{
  if (std::regex_search(expr, std::regex("\\(and"))) {return AND;}
  if (std::regex_search(expr, std::regex("\\(or"))) {return OR;}
  if (std::regex_search(expr, std::regex("\\(not"))) {return NOT;}

  return PREDICATE;
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

  int start = wexpr.find("(", 0);
  int it = start + 1;
  int balance = 1;

  while (it < wexpr.size()) {
    if (wexpr[it] == '(') {balance++;}
    if (wexpr[it] == ')') {balance--;}

    if (balance == 0) {
      ret.push_back(wexpr.substr(start, it - start + 1));

      start = wexpr.find("(", it);

      if (start == std::string::npos) {break;}
      it = start + 1;
      balance = 1;
    }

    it++;
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

std::vector<plansys2::Param> getPredicateParams(const std::string & expr)
{
  std::vector<plansys2::Param> ret;

  std::string wstring(expr);

  size_t start = 0, end = 0;
  end = wstring.find(" ", start);
  wstring = wstring.substr(end + 1, wstring.size() - end - 2);

  start = 0;
  while (end != std::string::npos) {
    end = wstring.find(" ", start);

    plansys2::Param param;
    param.name = wstring.substr(
      start,
      (end == std::string::npos) ? std::string::npos : end - start);

    ret.push_back(param);

    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

std::shared_ptr<TreeNode> get_tree_node(const std::string & expr)
{
  std::string wexpr = getReducedString(expr);
  NodeType type = getType(wexpr);

  if (wexpr == "(and)") {
    return nullptr;
  }
  switch (type) {
    case AND: {
        std::shared_ptr<plansys2::AndNode> pn_and = std::make_shared<plansys2::AndNode>();

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          pn_and->ops.push_back(get_tree_node(e));
        }

        return pn_and;
      }

    case OR: {
        std::shared_ptr<plansys2::OrNode> pn_or = std::make_shared<plansys2::OrNode>();

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          pn_or->ops.push_back(get_tree_node(e));
        }

        return pn_or;
      }

    case NOT: {
        std::shared_ptr<plansys2::NotNode> pn_not = std::make_shared<plansys2::NotNode>();

        std::vector<std::string> subexprs = getSubExpr(wexpr);
        pn_not->op = get_tree_node(subexprs[0]);

        return pn_not;
      }

    case PREDICATE: {
        std::shared_ptr<plansys2::PredicateNode> pred =
          std::make_shared<plansys2::PredicateNode>();

        pred->predicate_.name = getPredicateName(wexpr);
        pred->predicate_.parameters = getPredicateParams(wexpr);

        return pred;
      }

    default:
      std::cerr << "get_tree_node: Error parsing expresion [" << wexpr << "]" << std::endl;
  }

  return nullptr;
}

}  // namespace plansys2
