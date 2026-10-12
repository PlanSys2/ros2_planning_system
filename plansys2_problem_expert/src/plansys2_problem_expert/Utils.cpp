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

#include <algorithm>
#include <cmath>
#include <tuple>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <utility>

#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

namespace
{

std::tuple<bool, bool, double> evaluate_node(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  bool use_state,
  uint32_t node_id,
  bool negate)
{
  if (tree.nodes.empty()) {  // No expression
    return std::make_tuple(true, true, 0);
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND:
    case plansys2_msgs::msg::Node::OR: {
        // Children come back already negated: by De Morgan, (not (and a b)) is true if any
        // negated child is, and (not (or a b)) if all of them are
        const bool all = (tree.nodes[node_id].node_type == plansys2_msgs::msg::Node::AND) != negate;
        bool success = true;
        bool truth_value = all;

        for (auto & child_id : tree.nodes[node_id].children) {
          std::tuple<bool, bool, double> result =
            evaluate_node(
            tree, problem_client, predicates, functions, apply, use_state, child_id,
            negate);
          success = success && std::get<0>(result);
          truth_value = all ? (truth_value && std::get<1>(result)) :
            (truth_value || std::get<1>(result));
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case plansys2_msgs::msg::Node::NOT: {
        return evaluate_node(
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
                tree.nodes[node_id], true));
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
                  tree.nodes[node_id], true)) != predicates.end());
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
              tree.nodes[node_id], true));
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
        // Numbers are not negated: only the comparison is
        std::tuple<bool, bool, double> left = evaluate_node(
          tree, problem_client, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[0], false);
        std::tuple<bool, bool, double> right = evaluate_node(
          tree, problem_client, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[1], false);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        switch (tree.nodes[node_id].expression_type) {
          case plansys2_msgs::msg::Node::COMP_GE:
            if (std::get<2>(left) >= std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_GT:
            if (std::get<2>(left) > std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LE:
            if (std::get<2>(left) <= std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LT:
            if (std::get<2>(left) < std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_EQ: {
              auto c_t = plansys2_msgs::msg::Node::CONSTANT;
              auto p_t = plansys2_msgs::msg::Node::PARAMETER;
              auto c0 = tree.nodes[tree.nodes[node_id].children[0]];
              auto c1 = tree.nodes[tree.nodes[node_id].children[1]];
              auto c0_type = c0.node_type;
              auto c1_type = c1.node_type;
              if ((c0_type == c_t || c0_type == p_t) && (c1_type == c_t || c1_type == p_t)) {
                std::string c0_name = (c0_type == p_t) ? c0.parameters[0].name : c0.name;
                std::string c1_name = (c1_type == p_t) ? c1.parameters[0].name : c1.name;
                return std::make_tuple(
                  true,
                  negate ^ (c1_name == c0_name),
                  0);
              }
              if (c0_type == c_t || c0_type == p_t || c1_type == c_t || c1_type == p_t) {
                break;  // an object is not equal to a number
              }
              // Numbers, functions and arithmetic: equal up to rounding
              const double l = std::get<2>(left);
              const double r = std::get<2>(right);
              const bool equal =
                std::abs(l - r) <= 1e-9 * std::max({1.0, std::abs(l), std::abs(r)});
              return std::make_tuple(true, negate ^ equal, 0);
            }
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
        if (tree.nodes[node_id].children.size() < 2) {
          // total-cost modifiers carry only the value: costs are not tracked here
          return std::make_tuple(true, true, 0);
        }
        std::tuple<bool, bool, double> left = evaluate_node(
          tree, problem_client, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[0], false);
        std::tuple<bool, bool, double> right = evaluate_node(
          tree, problem_client,
          predicates, functions, apply, use_state, tree.nodes[node_id].children[1],
          false);

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
          uint32_t left_id = tree.nodes[node_id].children[0];
          if (use_state) {
            auto it =
              std::find_if(
              functions.begin(), functions.end(),
              std::bind(
                &parser::pddl::checkNodeEquality, std::placeholders::_1,
                tree.nodes[left_id], true));
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

    case plansys2_msgs::msg::Node::CONSTANT: {
        if (tree.nodes[node_id].name.size() > 0) {
          return std::make_tuple(true, true, 0);
        }
        return std::make_tuple(true, false, 0);
      }

    case plansys2_msgs::msg::Node::PARAMETER: {
        if (tree.nodes[node_id].parameters.size() > 0 &&
          tree.nodes[node_id].parameters[0].name.front() != '?')
        {
          return std::make_tuple(true, true, 0);
        }
        [[fallthrough]];
      }

    case plansys2_msgs::msg::Node::EXISTS:
    case plansys2_msgs::msg::Node::FORALL: {
        const auto & node = tree.nodes[node_id];

        // The objects each variable ranges over: those of its type or of a subtype. In the
        // state, an object's types are those its predicates and functions give it; an
        // object of unknown type is not discarded
        std::map<std::string, std::set<std::string>> object_types;
        if (use_state == false) {
          for (const auto & instance : problem_client->getInstances()) {
            object_types[instance.name].insert(instance.type);
          }
        } else {
          auto collect = [&object_types](const plansys2_msgs::msg::Node & n) {
              for (const auto & p : n.parameters) {
                auto & types = object_types[p.name];
                if (!p.type.empty()) {types.insert(p.type);}
              }
            };
          std::for_each(predicates.begin(), predicates.end(), collect);
          std::for_each(functions.begin(), functions.end(), collect);
        }

        std::vector<std::vector<std::string>> parameters_vector;
        for (const auto & var : node.parameters) {
          std::vector<std::string> candidates;
          for (const auto & [name, types] : object_types) {
            bool fits = var.type.empty() || types.empty();
            for (const auto & type : types) {
              fits = fits || type == var.type ||
                std::find(var.sub_types.begin(), var.sub_types.end(), type) != var.sub_types.end();
            }
            if (fits) {candidates.push_back(name);}
          }
          parameters_vector.push_back(candidates);
        }

        std::vector<std::vector<std::string>> possible_parameters_values;
        std::vector<std::string> aux;
        plansys2::cart_product(
          possible_parameters_values, aux, parameters_vector.begin(), parameters_vector.end());

        // A forall in an effect applies its body for every object
        const bool is_forall = node.node_type == plansys2_msgs::msg::Node::FORALL;
        if (apply && is_forall) {
          bool success = true;
          for (const auto & parameters_values : possible_parameters_values) {
            std::map<std::string, std::string> replace;
            for (size_t i = 0; i < node.parameters.size(); i++) {
              replace[node.parameters[i].name] = parameters_values[i];
            }
            auto tree_replaced = plansys2::replace_children_param(tree, node_id, replace);
            auto result = evaluate_node(
              tree_replaced, problem_client, predicates, functions, apply, use_state,
              tree_replaced.nodes[node_id].children[0], negate);
            success = success && std::get<0>(result);
          }
          return std::make_tuple(success, true, 0);
        }

        // The body comes back negated under not, so (not (exists ...)) needs it true for all
        // objects, and (not (forall ...)) for one
        const bool need_all = is_forall != negate;
        bool success = true;
        for (const auto & parameters_values : possible_parameters_values) {
          std::map<std::string, std::string> replace;
          for (size_t i = 0; i < node.parameters.size(); i++) {
            replace[node.parameters[i].name] = parameters_values[i];
          }
          auto tree_replaced = plansys2::replace_children_param(tree, node_id, replace);
          std::tuple<bool, bool, double> result = evaluate_node(
            tree_replaced,
            problem_client,
            predicates,
            functions,
            false,
            use_state,
            tree_replaced.nodes[node_id].children[0],
            negate);
          success = success && std::get<0>(result);
          if (need_all && !std::get<1>(result)) {
            return std::make_tuple(success, false, 0);
          }
          if (!need_all && std::get<1>(result)) {
            return std::make_tuple(success, true, 0);
          }
        }
        // No object left: everything holds for none, and none exists
        return std::make_tuple(success, need_all, 0);
      }

    case plansys2_msgs::msg::Node::IMPLY: {
        // (imply a b) is (or (not a) b); under not, (and a (not b))
        std::tuple<bool, bool, double> left = evaluate_node(
          tree, problem_client, predicates, functions, false, use_state,
          tree.nodes[node_id].children[0], !negate);
        std::tuple<bool, bool, double> right = evaluate_node(
          tree, problem_client, predicates, functions, false, use_state,
          tree.nodes[node_id].children[1], negate);
        const bool truth_value = negate ?
          (std::get<1>(left) && std::get<1>(right)) :
          (std::get<1>(left) || std::get<1>(right));
        return std::make_tuple(std::get<0>(left) && std::get<0>(right), truth_value, 0);
      }

    default:
      std::cerr << "evaluate: Error parsing expresion [" <<
        parser::pddl::toString(tree, node_id) << "]" << std::endl;
  }

  return std::make_tuple(false, false, 0);
}

}  // namespace

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  bool use_state,
  uint32_t node_id,
  bool negate)
{
  if (tree.nodes.empty()) {  // No expression
    return std::make_tuple(true, true, 0);
  }

  // Trees may come from outside (e.g. services): walk them only if they are sound
  std::string error;
  if (node_id >= tree.nodes.size() || !parser::pddl::validateTree(tree, error)) {
    std::cerr << "evaluate: malformed tree: " <<
      (error.empty() ? "node id out of range" : error) << std::endl;
    return std::make_tuple(false, false, 0);
  }

  return evaluate_node(
    tree, problem_client, predicates, functions, apply, use_state, node_id, negate);
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

plansys2_msgs::msg::Tree replace_children_param(
  const plansys2_msgs::msg::Tree & tree,
  const uint32_t & node_id,
  const std::map<std::string, std::string> & replace)
{
  plansys2_msgs::msg::Tree new_tree = tree;
  if (tree.nodes[node_id].children.size() > 0) {
    for (auto & child_id : tree.nodes[node_id].children) {
      new_tree = replace_children_param(new_tree, child_id, replace);
    }
  }

  for (size_t i = 0; i < tree.nodes[node_id].parameters.size(); i++) {
    if (replace.find(tree.nodes[node_id].parameters[i].name) != replace.end()) {
      new_tree.nodes[node_id].parameters[i].name = replace.at(
        tree.nodes[node_id].parameters[i].name);
    }
  }
  return new_tree;
}

void cart_product(
  std::vector<std::vector<std::string>> & rvvi,  // final result
  std::vector<std::string> & rvi,  // current result
  std::vector<std::vector<std::string>>::const_iterator me,  // current input
  std::vector<std::vector<std::string>>::const_iterator end)  // final input
{
  if (me == end) {
    rvvi.push_back(rvi);
    return;
  }

  const std::vector<std::string> & mevi = *me;
  for (std::vector<std::string>::const_iterator it = mevi.begin(); it != mevi.end(); it++) {
    rvi.push_back(*it);
    cart_product(rvvi, rvi, me + 1, end);
    rvi.pop_back();
  }
}

}  // namespace plansys2
