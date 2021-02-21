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

#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

std::tuple<bool, bool, double> evaluate(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions,
  bool apply,
  bool use_state,
  bool negate)
{
  if (node == nullptr) {  // No expression
    return std::make_tuple(true, true, 0);
  }

  switch (node->type_) {
    case parser::pddl::tree::AND: {
        std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
          std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(node);
        bool success = true;
        bool truth_value = true;

        for (const auto & op : pn_and->ops) {
          std::tuple<bool, bool, double> result =
            evaluate(op, problem_client, predicates, functions, apply, use_state, negate);
          success = success && std::get<0>(result);
          truth_value = truth_value && std::get<1>(result);
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case parser::pddl::tree::OR: {
        std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
          std::dynamic_pointer_cast<parser::pddl::tree::OrNode>(node);
        bool success = true;
        bool truth_value = false;

        for (const auto & op : pn_or->ops) {
          std::tuple<bool, bool, double> result =
            evaluate(op, problem_client, predicates, functions, apply, use_state, negate);
          success = success && std::get<0>(result);
          truth_value = truth_value || std::get<1>(result);
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case parser::pddl::tree::NOT: {
        std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
          std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(node);

        return evaluate(
          pn_not->op, problem_client, predicates, functions, apply, use_state,
          !negate);
      }

    case parser::pddl::tree::PREDICATE: {
        std::shared_ptr<parser::pddl::tree::PredicateNode> pred =
          std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(node);

        bool success = true;
        bool value = true;

        if (apply) {
          if (use_state) {
            if (negate) {
              predicates.erase(pred->toString());
              value = false;
            } else {
              predicates.insert(pred->toString());
            }
          } else {
            if (negate) {
              success = success && problem_client->removePredicate(pred->predicate_);
              value = false;
            } else {
              success = success && problem_client->addPredicate(pred->predicate_);
            }
          }
        } else {
          // negate | exist | output
          //   F    |   F   |   F
          //   F    |   T   |   T
          //   T    |   F   |   T
          //   T    |   T   |   F
          if (use_state) {
            value = negate ^ (predicates.find(pred->toString()) != predicates.end());
          } else {
            value = negate ^ problem_client->existPredicate(pred->predicate_);
          }
        }

        return std::make_tuple(success, value, 0);
      }

    case parser::pddl::tree::FUNCTION: {
        std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(node);

        bool success = true;
        double value = 0;

        if (use_state) {
          value = functions[func_node->toString()];
        } else {
          std::optional<parser::pddl::tree::Function> func =
            problem_client->getFunction(func_node->toString());

          if (func.has_value()) {
            value = func.value().value;
          } else {
            success = false;
          }
        }

        return std::make_tuple(success, false, value);
      }

    case parser::pddl::tree::EXPRESSION: {
        std::shared_ptr<parser::pddl::tree::ExpressionNode> expr =
          std::dynamic_pointer_cast<parser::pddl::tree::ExpressionNode>(node);

        std::tuple<bool, bool, double> left = evaluate(
          expr->ops[0], problem_client, predicates,
          functions, apply, use_state, negate);
        std::tuple<bool, bool, double> right = evaluate(
          expr->ops[1], problem_client, predicates,
          functions, apply, use_state, negate);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        switch (expr->expr_type) {
          case parser::pddl::tree::COMP_GE:
            if (std::get<2>(left) >= std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case parser::pddl::tree::COMP_GT:
            if (std::get<2>(left) > std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case parser::pddl::tree::COMP_LE:
            if (std::get<2>(left) <= std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case parser::pddl::tree::COMP_LT:
            if (std::get<2>(left) < std::get<2>(right)) {
              return std::make_tuple(true, true, 0);
            } else {
              return std::make_tuple(true, false, 0);
            }
            break;
          case parser::pddl::tree::ARITH_MULT:
            return std::make_tuple(true, false, std::get<2>(left) * std::get<2>(right));
            break;
          case parser::pddl::tree::ARITH_DIV:
            if (std::abs(std::get<2>(right)) > 1e-5) {
              return std::make_tuple(true, false, std::get<2>(left) / std::get<2>(right));
            } else {
              // Division by zero not allowed.
              return std::make_tuple(false, false, 0);
            }
            break;
          default:
            break;
        }

        return std::make_tuple(false, false, 0);
      }

    case parser::pddl::tree::FUNCTION_MODIFIER: {
        std::shared_ptr<parser::pddl::tree::FunctionModifierNode> func_mod =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionModifierNode>(node);

        std::tuple<bool, bool, double> left = evaluate(
          func_mod->ops[0], problem_client, predicates,
          functions, apply, use_state, negate);
        std::tuple<bool, bool, double> right = evaluate(
          func_mod->ops[1], problem_client,
          predicates, functions, apply, use_state,
          negate);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(func_mod->ops[0]);

        bool success = true;
        double value = 0;

        switch (func_mod->modifier_type) {
          case parser::pddl::tree::ASSIGN:
            value = std::get<2>(right);
            break;
          case parser::pddl::tree::INCREASE:
            value = std::get<2>(left) + std::get<2>(right);
            break;
          case parser::pddl::tree::DECREASE:
            value = std::get<2>(left) - std::get<2>(right);
            break;
          case parser::pddl::tree::SCALE_UP:
            value = std::get<2>(left) * std::get<2>(right);
            break;
          case parser::pddl::tree::SCALE_DOWN:
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
          if (use_state) {
            functions[func_node->toString()] = value;
          } else {
            std::stringstream ss;
            ss << "(= " << func_node->toString() << " " << value << ")";
            problem_client->updateFunction(parser::pddl::tree::Function(ss.str()));
          }
        }

        return std::make_tuple(success, false, value);
      }

    case parser::pddl::tree::NUMBER: {
        std::shared_ptr<parser::pddl::tree::NumberNode> num =
          std::dynamic_pointer_cast<parser::pddl::tree::NumberNode>(node);

        return std::make_tuple(true, true, num->value_);
      }

    default:
      std::cerr << "evaluate: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return std::make_tuple(false, false, 0);
}

std::tuple<bool, bool, double> evaluate(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply)
{
  std::set<std::string> predicates;
  std::map<std::string, double> functions;
  return evaluate(node, problem_client, predicates, functions, apply);
}

std::tuple<bool, bool, double> evaluate(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions,
  bool apply)
{
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client;
  return evaluate(node, problem_client, predicates, functions, apply, true);
}

bool check(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  std::tuple<bool, bool, double> ret = evaluate(node, problem_client);

  return std::get<1>(ret);
}

bool check(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions)
{
  std::tuple<bool, bool, double> ret = evaluate(node, predicates, functions);

  return std::get<1>(ret);
}

bool apply(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  std::tuple<bool, bool, double> ret = evaluate(node, problem_client, true);

  return std::get<0>(ret);
}

bool apply(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions)
{
  std::tuple<bool, bool, double> ret = evaluate(node, predicates, functions, true);

  return std::get<0>(ret);
}

std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> get_subtrees(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node)
{
  if (node == nullptr) {  // No expression
    return {};
  }

  switch (node->type_) {
    case parser::pddl::tree::AND: {
        std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
          std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(node);
        return pn_and->ops;
      }

    default:
      std::cerr << "get_subtrees: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return {};
}

std::shared_ptr<parser::pddl::tree::DurativeAction> get_action_from_string(
  const std::string & action_expr,
  std::shared_ptr<plansys2::DomainExpertClient> domain_client)
{
  auto action_output = std::make_shared<parser::pddl::tree::DurativeAction>();

  action_output->name = get_name(action_expr);

  action_output->parameters.clear();
  for (const auto & param : get_params(action_expr)) {
    action_output->parameters.push_back(parser::pddl::tree::Param{param, ""});
  }

  auto action = domain_client->getAction(action_output->name);
  auto durative_action = domain_client->getDurativeAction(action_output->name);

  if (action) {
    auto at_start_req = action.value().preconditions.toString();
    auto at_end_eff = action.value().effects.toString();

    for (size_t i = 0; i < action_output->parameters.size(); i++) {
      std::string pattern = "?" + std::to_string(i);
      size_t pos;
      while ((pos = at_start_req.find(pattern)) != std::string::npos) {
        at_start_req.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
      while ((pos = at_end_eff.find(pattern)) != std::string::npos) {
        at_end_eff.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
    }

    action_output->at_start_requirements.fromString(at_start_req);
    action_output->over_all_requirements.fromString("");
    action_output->at_end_requirements.fromString("");
    action_output->at_start_effects.fromString("");
    action_output->at_end_effects.fromString(at_end_eff);

    return action_output;
  }

  if (durative_action) {
    auto at_start_req = durative_action.value().at_start_requirements.toString();
    auto over_all_req = durative_action.value().over_all_requirements.toString();
    auto at_end_req = durative_action.value().at_end_requirements.toString();
    auto at_start_eff = durative_action.value().at_start_effects.toString();
    auto at_end_eff = durative_action.value().at_end_effects.toString();

    for (size_t i = 0; i < action_output->parameters.size(); i++) {
      std::string pattern = "?" + std::to_string(i);
      size_t pos;
      while ((pos = at_start_req.find(pattern)) != std::string::npos) {
        at_start_req.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
      while ((pos = over_all_req.find(pattern)) != std::string::npos) {
        over_all_req.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
      while ((pos = at_end_req.find(pattern)) != std::string::npos) {
        at_end_req.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
      while ((pos = at_start_eff.find(pattern)) != std::string::npos) {
        at_start_eff.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
      while ((pos = at_end_eff.find(pattern)) != std::string::npos) {
        at_end_eff.replace(pos, pattern.length(), action_output->parameters[i].name);
      }
    }

    action_output->at_start_requirements.fromString(at_start_req);
    action_output->over_all_requirements.fromString(over_all_req);
    action_output->at_end_requirements.fromString(at_end_req);
    action_output->at_start_effects.fromString(at_start_eff);
    action_output->at_end_effects.fromString(at_end_eff);

    return action_output;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("rclcpp"), "Action [%s] not found",
    action_output->name.c_str());
  return nullptr;
}

std::vector<std::string> get_params(const std::string & action_expr)
{
  std::vector<std::string> ret;

  std::string working_action_expr = parser::pddl::tree::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  working_action_expr = working_action_expr.substr(delim + 1);

  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = working_action_expr.find(" ", start);
    auto param = working_action_expr.substr(
      start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

std::string get_name(const std::string & action_expr)
{
  std::string working_action_expr = parser::pddl::tree::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  return working_action_expr.substr(0, delim);
}

}  // namespace plansys2
