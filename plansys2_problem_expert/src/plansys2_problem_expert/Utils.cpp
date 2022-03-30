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

#include <tuple>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <utility>

#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.h"

namespace plansys2
{

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  bool use_state,
  uint8_t node_id,
  bool negate)
{
  if (tree.nodes.empty()) {  // No expression
    return std::make_tuple(true, true, 0);
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND: {
        bool success = true;
        bool truth_value = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          std::tuple<bool, bool, double> result =
            evaluate(
            tree, problem_client, predicates, functions, apply, use_state, child_id,
            negate);
          success = success && std::get<0>(result);
          truth_value = truth_value && std::get<1>(result);
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case plansys2_msgs::msg::Node::OR: {
        bool success = true;
        bool truth_value = false;

        for (auto & child_id : tree.nodes[node_id].children) {
          std::tuple<bool, bool, double> result =
            evaluate(
            tree, problem_client, predicates, functions, apply, use_state, child_id,
            negate);
          success = success && std::get<0>(result);
          truth_value = truth_value || std::get<1>(result);
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case plansys2_msgs::msg::Node::NOT: {
        return evaluate(
          tree, problem_client, predicates, functions, apply, use_state,
          tree.nodes[node_id].children[0],
          !negate);
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        bool success = true;
        bool value = true;

        if (apply) {
          if (use_state) {
            auto it =
              std::find_if(
              predicates.begin(), predicates.end(),
              std::bind(
                &parser::pddl::checkNodeEquality, std::placeholders::_1,
                tree.nodes[node_id]));
            if (negate) {
              if (it != predicates.end()) {
                predicates.erase(it);
              }
              value = false;
            } else {
              if (it == predicates.end()) {
                predicates.push_back(tree.nodes[node_id]);
              }
            }
          } else {
            if (negate) {
              success = success && problem_client->removePredicate(tree.nodes[node_id]);
              value = false;
            } else {
              success = success && problem_client->addPredicate(tree.nodes[node_id]);
            }
          }
        } else {
          // negate | exist | output
          //   F    |   F   |   F
          //   F    |   T   |   T
          //   T    |   F   |   T
          //   T    |   T   |   F
          if (use_state) {
            value = negate ^
              (std::find_if(
                predicates.begin(), predicates.end(),
                std::bind(
                  &parser::pddl::checkNodeEquality, std::placeholders::_1,
                  tree.nodes[node_id])) != predicates.end());
          } else {
            value = negate ^ problem_client->existPredicate(tree.nodes[node_id]);
          }
        }

        return std::make_tuple(success, value, 0);
      }

    case plansys2_msgs::msg::Node::FUNCTION: {
        bool success = true;
        double value = 0;

        if (use_state) {
          auto it =
            std::find_if(
            functions.begin(), functions.end(),
            std::bind(
              &parser::pddl::checkNodeEquality, std::placeholders::_1,
              tree.nodes[node_id]));
          if (it != functions.end()) {
            value = it->value;
          } else {
            success = false;
          }
        } else {
          std::optional<plansys2_msgs::msg::Node> func =
            problem_client->getFunction(parser::pddl::toString(tree, node_id));

          if (func.has_value()) {
            value = func.value().value;
          } else {
            success = false;
          }
        }

        return std::make_tuple(success, false, value);
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        std::tuple<bool, bool, double> left = evaluate(
          tree, problem_client, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[0], negate);
        std::tuple<bool, bool, double> right = evaluate(
          tree, problem_client, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[1], negate);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        switch (tree.nodes[node_id].expression_type) {
          case plansys2_msgs::msg::Node::COMP_GE:
            if (std::get<2>(left) >= std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_GT:
            if (std::get<2>(left) > std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LE:
            if (std::get<2>(left) <= std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LT:
            if (std::get<2>(left) < std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_EQ:
            if (std::get<2>(left) == std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;

          case plansys2_msgs::msg::Node::ARITH_MULT:
            return std::make_tuple(true, false, std::get<2>(left) * std::get<2>(right));
            break;
          case plansys2_msgs::msg::Node::ARITH_DIV:
            if (std::abs(std::get<2>(right)) > 1e-5) {
              return std::make_tuple(true, false, std::get<2>(left) / std::get<2>(right));
            } else {
              // Division by zero not allowed.
              return std::make_tuple(false, false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::ARITH_ADD:
            return std::make_tuple(true, false, std::get<2>(left) + std::get<2>(right));
            break;
          case plansys2_msgs::msg::Node::ARITH_SUB:
            return std::make_tuple(true, false, std::get<2>(left) - std::get<2>(right));
            break;
          default:
            break;
        }

        return std::make_tuple(false, false, 0);
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        std::tuple<bool, bool, double> left = evaluate(
          tree, problem_client, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[0], negate);
        std::tuple<bool, bool, double> right = evaluate(
          tree, problem_client,
          predicates, functions, apply, use_state, tree.nodes[node_id].children[1],
          negate);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        bool success = true;
        double value = 0;

        switch (tree.nodes[node_id].modifier_type) {
          case plansys2_msgs::msg::Node::ASSIGN:
            value = std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::INCREASE:
            value = std::get<2>(left) + std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::DECREASE:
            value = std::get<2>(left) - std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::SCALE_UP:
            value = std::get<2>(left) * std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::SCALE_DOWN:
            // Division by zero not allowed.
            if (std::abs(std::get<2>(right)) > 1e-5) {
              value = std::get<2>(left) / std::get<2>(right);
            } else {
              success = false;
            }
            break;
          default:
            success = false;
            break;
        }

        if (success && apply) {
          uint8_t left_id = tree.nodes[node_id].children[0];
          if (use_state) {
            auto it =
              std::find_if(
              functions.begin(), functions.end(),
              std::bind(
                &parser::pddl::checkNodeEquality, std::placeholders::_1,
                tree.nodes[left_id]));
            if (it != functions.end()) {
              it->value = value;
            } else {
              success = false;
            }
          } else {
            std::stringstream ss;
            ss << "(= " << parser::pddl::toString(tree, left_id) << " " << value << ")";
            problem_client->updateFunction(parser::pddl::fromStringFunction(ss.str()));
          }
        }

        return std::make_tuple(success, false, value);
      }

    case plansys2_msgs::msg::Node::NUMBER: {
        return std::make_tuple(true, true, tree.nodes[node_id].value);
      }

    default:
      std::cerr << "evaluate: Error parsing expresion [" <<
        parser::pddl::toString(tree, node_id) << "]" << std::endl;
  }

  return std::make_tuple(false, false, 0);
}

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply,
  uint32_t node_id)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  return evaluate(tree, problem_client, predicates, functions, apply, false, node_id);
}

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  uint32_t node_id)
{
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client;
  return evaluate(tree, problem_client, predicates, functions, apply, true, node_id);
}

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, problem_client, false, node_id);

  return std::get<1>(ret);
}

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, predicates, functions, false, node_id);

  return std::get<1>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, problem_client, true, node_id);

  return std::get<0>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, predicates, functions, true, node_id);

  return std::get<0>(ret);
}

std::pair<std::string, int> parse_action(const std::string & input)
{
  std::string action = parser::pddl::getReducedString(input);
  int time = -1;

  size_t delim = action.find(":");
  if (delim != std::string::npos) {
    time = std::stoi(action.substr(delim + 1, action.length() - delim - 1));
    action.erase(action.begin() + delim, action.end());
  }

  action.erase(0, 1);  // remove initial (
  action.pop_back();  // remove last )

  return std::make_pair(action, time);
}

std::string get_action_expression(const std::string & input)
{
  auto action = parse_action(input);
  return action.first;
}

int get_action_time(const std::string & input)
{
  auto action = parse_action(input);
  return action.second;
}

std::string get_action_name(const std::string & input)
{
  auto expr = get_action_expression(input);
  size_t delim = expr.find(" ");
  return expr.substr(0, delim);
}

std::vector<std::string> get_action_params(const std::string & input)
{
  std::vector<std::string> ret;

  auto expr = get_action_expression(input);

  size_t delim = expr.find(" ");
  if (delim != std::string::npos) {
    expr.erase(expr.begin(), expr.begin() + delim + 1);
  }

  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = expr.find(" ", start);
    auto param = expr.substr(
      start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

}  // namespace plansys2
