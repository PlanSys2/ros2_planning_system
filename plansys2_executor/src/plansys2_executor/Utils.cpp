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

#include <memory>
#include <string>
#include <vector>

#include "plansys2_executor/Utils.hpp"

namespace plansys2
{

bool check(
  const std::shared_ptr<plansys2::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  if (node == nullptr) {  // No req expression
    return true;
  }

  switch (node->type_) {
    case AND: {
        std::shared_ptr<plansys2::AndNode> pn_and =
          std::dynamic_pointer_cast<plansys2::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          ret = ret && check(op, problem_client);
        }
        return ret;
      }

    case OR: {
        std::shared_ptr<plansys2::OrNode> pn_or =
          std::dynamic_pointer_cast<plansys2::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          ret = ret || check(op, problem_client);
        }
        return ret;
      }

    case NOT: {
        std::shared_ptr<plansys2::NotNode> pn_not =
          std::dynamic_pointer_cast<NotNode>(node);

        return !check(pn_not->op, problem_client);
      }

    case PREDICATE: {
        std::shared_ptr<plansys2::PredicateNode> pred =
          std::dynamic_pointer_cast<PredicateNode>(node);

        return problem_client->existPredicate(pred->predicate_);
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return false;
}


bool apply(
  const std::shared_ptr<plansys2::TreeNode> node,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, bool negate)
{
  if (node == nullptr) {  // No apply expression
    return true;
  }

  switch (node->type_) {
    case AND: {
        std::shared_ptr<plansys2::AndNode> pn_and =
          std::dynamic_pointer_cast<plansys2::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          ret = ret && apply(op, problem_client, negate);
        }
        return ret;
      }

    case OR: {
        std::shared_ptr<plansys2::OrNode> pn_or =
          std::dynamic_pointer_cast<plansys2::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          ret = ret && apply(op, problem_client, negate);
        }
        return ret;
      }

    case NOT: {
        std::shared_ptr<plansys2::NotNode> pn_not =
          std::dynamic_pointer_cast<NotNode>(node);

        return apply(pn_not->op, problem_client, !negate);
      }

    case PREDICATE: {
        std::shared_ptr<plansys2::PredicateNode> pred =
          std::dynamic_pointer_cast<PredicateNode>(node);

        if (negate) {
          auto success = problem_client->removePredicate(pred->predicate_);
          return success;
        } else {
          auto success = problem_client->addPredicate(pred->predicate_);
          return success;
        }
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return false;
}

std::shared_ptr<DurativeAction> get_action_from_string(
  const std::string & action_expr,
  std::shared_ptr<plansys2::DomainExpertClient> domain_client)
{
  auto action_output = std::make_shared<DurativeAction>();

  action_output->name = get_name(action_expr);

  action_output->parameters.clear();
  for (const auto & param : get_params(action_expr)) {
    action_output->parameters.push_back(Param{param, ""});
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

  std::string working_action_expr = getReducedString(action_expr);
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
  std::string working_action_expr = getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  return working_action_expr.substr(0, delim);
}

}  // namespace plansys2
