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

#include "plansys2_executor/Utils.hpp"

namespace plansys2
{

std::tuple<bool, double> check(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  if (node == nullptr) {  // No req expression
    return std::make_tuple(true, 0);
  }

  switch (node->type_) {
    case parser::pddl::tree::AND: {
        std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
          std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          std::tuple<bool, double> result = check(op, problem_client);
          ret = ret && std::get<0>(result);
        }
        return std::make_tuple(ret, 0);
      }

    case parser::pddl::tree::OR: {
        std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
          std::dynamic_pointer_cast<parser::pddl::tree::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          std::tuple<bool, double> result = check(op, problem_client);
          ret = ret || std::get<0>(result);
        }
        return std::make_tuple(ret, 0);
      }

    case parser::pddl::tree::NOT: {
        std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
          std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(node);

        std::tuple<bool, double> result = check(pn_not->op, problem_client);
        return std::make_tuple(!std::get<0>(result), 0);
      }

    case parser::pddl::tree::PREDICATE: {
        std::shared_ptr<parser::pddl::tree::PredicateNode> pred =
          std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(node);

        return std::make_tuple(problem_client->existPredicate(pred->predicate_), 0);
      }

    case parser::pddl::tree::FUNCTION: {
        std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(node);

        std::optional<parser::pddl::tree::Function> func =
          problem_client->getFunction(func_node->toString());

        if (func.has_value()) {
          return std::make_tuple(true, func.value().value);
        }

        return std::make_tuple(false, 0);
      }

    case parser::pddl::tree::EXPRESSION: {
        std::shared_ptr<parser::pddl::tree::ExpressionNode> expr =
          std::dynamic_pointer_cast<parser::pddl::tree::ExpressionNode>(node);

        std::tuple<bool, double> left = check(expr->ops[0], problem_client);
        std::tuple<bool, double> right = check(expr->ops[1], problem_client);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, 0);
        }

        switch (expr->expr_type) {
          case parser::pddl::tree::COMP_GE:
            if (std::get<1>(left) >= std::get<1>(right)) {
              return std::make_tuple(true, 0);
            } else {
              return std::make_tuple(false, 0);
            }
            break;
          case parser::pddl::tree::COMP_GT:
            if (std::get<1>(left) > std::get<1>(right)) {
              return std::make_tuple(true, 0);
            } else {
              return std::make_tuple(false, 0);
            }
            break;
          case parser::pddl::tree::COMP_LE:
            if (std::get<1>(left) <= std::get<1>(right)) {
              return std::make_tuple(true, 0);
            } else {
              return std::make_tuple(false, 0);
            }
            break;
          case parser::pddl::tree::COMP_LT:
            if (std::get<1>(left) < std::get<1>(right)) {
              return std::make_tuple(true, 0);
            } else {
              return std::make_tuple(false, 0);
            }
            break;
          case parser::pddl::tree::ARITH_MULT:
            return std::make_tuple(true, std::get<1>(left) * std::get<1>(right));
            break;
          case parser::pddl::tree::ARITH_DIV:
            if (std::get<1>(right) > 1e-5) {
              return std::make_tuple(true, std::get<1>(left) / std::get<1>(right));
            } else {
              // Division by zero not allowed.
              return std::make_tuple(false, 0);
            }
            break;
          default:
            break;
        }

        return std::make_tuple(false, 0);
      }

    case parser::pddl::tree::FUNCTION_MODIFIER: {
        std::shared_ptr<parser::pddl::tree::FunctionModifierNode> func_mod =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionModifierNode>(node);

        std::tuple<bool, double> left = check(func_mod->ops[0], problem_client);
        std::tuple<bool, double> right = check(func_mod->ops[1], problem_client);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, 0);
        }

        switch (func_mod->modifier_type) {
          case parser::pddl::tree::ASSIGN:
            return std::make_tuple(true, std::get<1>(right));
            break;
          case parser::pddl::tree::INCREASE:
            return std::make_tuple(true, std::get<1>(left) + std::get<1>(right));
            break;
          case parser::pddl::tree::DECREASE:
            return std::make_tuple(true, std::get<1>(left) - std::get<1>(right));
            break;
          case parser::pddl::tree::SCALE_UP:
            return std::make_tuple(true, std::get<1>(left) * std::get<1>(right));
            break;
          case parser::pddl::tree::SCALE_DOWN:
            return std::make_tuple(true, std::get<1>(left) / std::get<1>(right));
            break;
          default:
            break;
        }

        return std::make_tuple(false, 0);
      }

    case parser::pddl::tree::NUMBER: {
        std::shared_ptr<parser::pddl::tree::NumberNode> num =
          std::dynamic_pointer_cast<parser::pddl::tree::NumberNode>(node);

        return std::make_tuple(true, num->value_);
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return std::make_tuple(false, 0);
}

std::tuple<bool, bool, double> apply(
  const std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, bool negate)
{
  if (node == nullptr) {  // No apply expression
    return std::make_tuple(true, false, 0);
  }

  switch (node->type_) {
    case parser::pddl::tree::AND: {
        std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
          std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(node);
        bool success = true;
        bool truth_value = true;

        for (const auto & op : pn_and->ops) {
          std::tuple<bool, bool, double> result = apply(op, problem_client, negate);
          success = success && std::get<0>(result);
          truth_value = truth_value && std::get<1>(result);
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case parser::pddl::tree::OR: {
        std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
          std::dynamic_pointer_cast<parser::pddl::tree::OrNode>(node);
        bool success = true;
        bool truth_value = true;

        for (const auto & op : pn_or->ops) {
          std::tuple<bool, bool, double> result = apply(op, problem_client, negate);
          success = success && std::get<0>(result);
          truth_value = truth_value || std::get<1>(result);
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case parser::pddl::tree::NOT: {
        std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
          std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(node);

        return apply(pn_not->op, problem_client, !negate);
      }

    case parser::pddl::tree::PREDICATE: {
        std::shared_ptr<parser::pddl::tree::PredicateNode> pred =
          std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(node);

        if (negate) {
          auto success = problem_client->removePredicate(pred->predicate_);
          return std::make_tuple(success, false, 0);
        } else {
          auto success = problem_client->addPredicate(pred->predicate_);
          return std::make_tuple(success, true, 0);
        }
      }

    case parser::pddl::tree::FUNCTION: {
        std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(node);

        std::optional<parser::pddl::tree::Function> func =
          problem_client->getFunction(func_node->toString());

        if (func.has_value()) {
          return std::make_tuple(true, true, func.value().value);
        }

        return std::make_tuple(false, false, 0);
      }

    case parser::pddl::tree::EXPRESSION: {
        std::shared_ptr<parser::pddl::tree::ExpressionNode> expr =
          std::dynamic_pointer_cast<parser::pddl::tree::ExpressionNode>(node);

        std::tuple<bool, bool, double> left = apply(expr->ops[0], problem_client, negate);
        std::tuple<bool, bool, double> right = apply(expr->ops[1], problem_client, negate);

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
            return std::make_tuple(true, true, std::get<2>(left) * std::get<2>(right));
            break;
          case parser::pddl::tree::ARITH_DIV:
            if (std::get<2>(right) > 1e-5) {
              return std::make_tuple(true, true, std::get<2>(left) / std::get<2>(right));
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

        std::tuple<bool, bool, double> left = apply(func_mod->ops[0], problem_client, negate);
        std::tuple<bool, bool, double> right = apply(func_mod->ops[1], problem_client, negate);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(func_mod->ops[0]);

        bool valid_modifier = true;
        double result = 0;

        switch (func_mod->modifier_type) {
          case parser::pddl::tree::ASSIGN:
            result = std::get<2>(right);
            break;
          case parser::pddl::tree::INCREASE:
            result = std::get<2>(left) + std::get<2>(right);
            break;
          case parser::pddl::tree::DECREASE:
            result = std::get<2>(left) - std::get<2>(right);
            break;
          case parser::pddl::tree::SCALE_UP:
            result = std::get<2>(left) * std::get<2>(right);
            break;
          case parser::pddl::tree::SCALE_DOWN:
            result = std::get<2>(left) / std::get<2>(right);
            break;
          default:
            valid_modifier = false;
            break;
        }

        if (valid_modifier) {
          std::stringstream ss;
          ss << "(= " << func_node->toString() << " " << result << ")";
          problem_client->updateFunction(parser::pddl::tree::Function(ss.str()));
          return std::make_tuple(true, true, result);
        }

        return std::make_tuple(false, false, 0);
      }

    case parser::pddl::tree::NUMBER: {
        std::shared_ptr<parser::pddl::tree::NumberNode> num =
          std::dynamic_pointer_cast<parser::pddl::tree::NumberNode>(node);

        return std::make_tuple(true, true, num->value_);
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return std::make_tuple(false, false, 0);
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

    auto at_start_req_construct = action.value().preconditions.construct();
    auto at_end_eff_construct = action.value().effects.construct();

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

    action_output->at_start_requirements.fromString(at_start_req, at_start_req_construct);
    action_output->over_all_requirements.fromString("", "");
    action_output->at_end_requirements.fromString("", "");
    action_output->at_start_effects.fromString("", "");
    action_output->at_end_effects.fromString(at_end_eff, at_end_eff_construct);

    return action_output;
  }

  if (durative_action) {
    auto at_start_req = durative_action.value().at_start_requirements.toString();
    auto over_all_req = durative_action.value().over_all_requirements.toString();
    auto at_end_req = durative_action.value().at_end_requirements.toString();
    auto at_start_eff = durative_action.value().at_start_effects.toString();
    auto at_end_eff = durative_action.value().at_end_effects.toString();

    auto at_start_req_construct = durative_action.value().at_start_requirements.construct();
    auto over_all_req_construct = durative_action.value().over_all_requirements.construct();
    auto at_end_req_construct = durative_action.value().at_end_requirements.construct();
    auto at_start_eff_construct = durative_action.value().at_start_effects.construct();
    auto at_end_eff_construct = durative_action.value().at_end_effects.construct();

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

    action_output->at_start_requirements.fromString(at_start_req, at_start_req_construct);
    action_output->over_all_requirements.fromString(over_all_req, over_all_req_construct);
    action_output->at_end_requirements.fromString(at_end_req, at_end_req_construct);
    action_output->at_start_effects.fromString(at_start_eff, at_start_eff_construct);
    action_output->at_end_effects.fromString(at_end_eff, at_end_eff_construct);

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
