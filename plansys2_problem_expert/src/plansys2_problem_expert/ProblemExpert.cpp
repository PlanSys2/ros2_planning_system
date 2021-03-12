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

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <set>
#include <map>

#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

ProblemExpert::ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert)
: domain_expert_(domain_expert)
{
}

bool
ProblemExpert::addInstance(const parser::pddl::tree::Instance & instance)
{
  if (!isValidType(instance.type)) {
    return false;
  } else if (existInstance(instance.name)) {
    return false;
  } else {
    instances_.push_back(instance);
    return true;
  }
}

std::vector<parser::pddl::tree::Instance>
ProblemExpert::getInstances()
{
  return instances_;
}

bool
ProblemExpert::removeInstance(const std::string & name)
{
  bool found = false;
  int i = 0;

  while (!found && i < instances_.size()) {
    if (instances_[i].name == name) {
      found = true;
      instances_.erase(instances_.begin() + i);
    }
    i++;
  }

  // (fmrico)ToDo: We should remove all goals containing the removed instance
  removeFunctionsReferencing(name);
  removePredicatesReferencing(name);

  return found;
}

std::optional<parser::pddl::tree::Instance>
ProblemExpert::getInstance(const std::string & instance_name)
{
  parser::pddl::tree::Instance ret;

  bool found = false;
  int i = 0;
  while (i < instances_.size() && !found) {
    if (instances_[i].name == instance_name) {
      found = true;
      ret = instances_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<parser::pddl::tree::Predicate>
ProblemExpert::getPredicates()
{
  return predicates_;
}

bool
ProblemExpert::addPredicate(const parser::pddl::tree::Predicate & predicate)
{
  if (!existPredicate(predicate)) {
    if (isValidPredicate(predicate)) {
      predicates_.push_back(predicate);
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

bool
ProblemExpert::removePredicate(const parser::pddl::tree::Predicate & predicate)
{
  bool found = false;
  int i = 0;

  if (!isValidPredicate(predicate)) {  // if predicate is not valid, error
    return false;
  }
  while (!found && i < predicates_.size()) {
    if (predicates_[i] == predicate) {
      found = true;
      predicates_.erase(predicates_.begin() + i);
    }
    i++;
  }

  return true;
}

std::optional<parser::pddl::tree::Predicate>
ProblemExpert::getPredicate(const std::string & expr)
{
  parser::pddl::tree::Predicate ret;
  parser::pddl::tree::Predicate pred(expr);

  bool found = false;
  size_t i = 0;
  while (i < predicates_.size() && !found) {
    if (predicates_[i].name == pred.name) {
      if (predicates_[i].parameters.size() == pred.parameters.size()) {
        found = true;
        for (size_t j = 0; j < predicates_[i].parameters.size(); ++j) {
          if (predicates_[i].parameters[j].name != pred.parameters[j].name) {
            found = false;
            break;
          }
        }
        if (found) {
          ret = predicates_[i];
        }
      }
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<parser::pddl::tree::Function>
ProblemExpert::getFunctions()
{
  return functions_;
}

bool
ProblemExpert::addFunction(const parser::pddl::tree::Function & function)
{
  if (!existFunction(function)) {
    if (isValidFunction(function)) {
      functions_.push_back(function);
      return true;
    } else {
      return false;
    }
  } else {
    return updateFunction(function);
  }
}

bool
ProblemExpert::removeFunction(const parser::pddl::tree::Function & function)
{
  bool found = false;
  int i = 0;

  if (!isValidFunction(function)) {  // if function is not valid, error
    return false;
  }
  while (!found && i < functions_.size()) {
    if (functions_[i] == function) {
      found = true;
      functions_.erase(functions_.begin() + i);
    }
    i++;
  }

  return true;
}

bool
ProblemExpert::updateFunction(const parser::pddl::tree::Function & function)
{
  if (existFunction(function)) {
    if (isValidFunction(function)) {
      removeFunction(function);
      functions_.push_back(function);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

std::optional<parser::pddl::tree::Function>
ProblemExpert::getFunction(const std::string & expr)
{
  parser::pddl::tree::Function ret;
  parser::pddl::tree::Function func(expr);

  bool found = false;
  size_t i = 0;
  while (i < functions_.size() && !found) {
    if (functions_[i].name == func.name) {
      if (functions_[i].parameters.size() == func.parameters.size()) {
        found = true;
        for (size_t j = 0; j < functions_[i].parameters.size(); ++j) {
          if (functions_[i].parameters[j].name != func.parameters[j].name) {
            found = false;
            break;
          }
        }
        if (found) {
          ret = functions_[i];
        }
      }
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

bool
ProblemExpert::removeFunctionsReferencing(const std::string & name)
{
  int i = 0;

  while (i < functions_.size()) {
    bool found = false;
    for (parser::pddl::tree::Param parameter : functions_[i].parameters) {
      if (parameter.name == name) {
        functions_.erase(functions_.begin() + i);
        found = true;
        break;
      }
    }
    if (!found) {
      i++;
    }
  }
  return false;
}

bool
ProblemExpert::removePredicatesReferencing(const std::string & name)
{
  int i = 0;

  while (i < predicates_.size()) {
    bool found = false;
    for (parser::pddl::tree::Param parameter : predicates_[i].parameters) {
      if (parameter.name == name) {
        predicates_.erase(predicates_.begin() + i);
        found = true;
        break;
      }
    }
    if (!found) {
      i++;
    }
  }
  return false;
}

parser::pddl::tree::Goal
ProblemExpert::getGoal()
{
  return goal_;
}

bool ProblemExpert::isGoalSatisfied(const parser::pddl::tree::Goal & goal)
{
  std::set<std::string> predicates;
  for (auto & predicate : predicates_) {
    predicates.insert(predicate.toString());
  }

  std::map<std::string, double> functions;
  for (auto & function : functions_) {
    functions.insert({function.toString(), function.value});
  }

  return check(goal.root_, predicates, functions);
}

bool
ProblemExpert::setGoal(const parser::pddl::tree::Goal & goal)
{
  if (isValidGoal(goal)) {
    goal_.clear();
    goal_ = goal;
    return true;
  } else {
    return false;
  }
}

bool
ProblemExpert::clearGoal()
{
  goal_.clear();
  return true;
}

bool
ProblemExpert::clearKnowledge()
{
  instances_.clear();
  predicates_.clear();
  functions_.clear();
  return true;
}

bool
ProblemExpert::isValidType(const std::string & type)
{
  auto valid_types = domain_expert_->getTypes();
  auto it = std::find(valid_types.begin(), valid_types.end(), type);

  return it != valid_types.end();
}

bool
ProblemExpert::existInstance(const std::string & name)
{
  bool found = false;
  int i = 0;

  while (!found && i < instances_.size()) {
    if (instances_[i].name == name) {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::existPredicate(const parser::pddl::tree::Predicate & predicate)
{
  bool found = false;
  int i = 0;

  while (!found && i < predicates_.size()) {
    if (predicates_[i].name == predicate.name &&
      predicates_[i].parameters == predicate.parameters)
    {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::existFunction(const parser::pddl::tree::Function & function)
{
  bool found = false;
  int i = 0;

  while (!found && i < functions_.size()) {
    if (functions_[i].name == function.name &&
      functions_[i].parameters == function.parameters)
    {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::isValidPredicate(const parser::pddl::tree::Predicate & predicate)
{
  bool valid = false;

  const std::optional<parser::pddl::tree::Predicate> & model_predicate =
    domain_expert_->getPredicate(predicate.name);
  if (model_predicate) {
    if (model_predicate.value().parameters.size() == predicate.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < predicate.parameters.size()) {
        auto arg_type = getInstance(predicate.parameters[i].name);

        if (!arg_type) {
          same_types = false;
        } else if (arg_type.value().type != model_predicate.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_predicate.value().parameters[i].subTypes) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool
ProblemExpert::isValidFunction(const parser::pddl::tree::Function & function)
{
  bool valid = false;

  const std::optional<parser::pddl::tree::Function> & model_function = domain_expert_->getFunction(
    function.name);
  if (model_function) {
    if (model_function.value().parameters.size() == function.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < function.parameters.size()) {
        auto arg_type = getInstance(function.parameters[i].name);

        if (!arg_type) {
          same_types = false;
        } else if (arg_type.value().type != model_function.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_function.value().parameters[i].subTypes) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool
ProblemExpert::isValidGoal(const parser::pddl::tree::Goal & goal)
{
  return checkPredicateTreeTypes(goal.root_, domain_expert_);
}

bool
ProblemExpert::checkPredicateTreeTypes(
  std::shared_ptr<parser::pddl::tree::TreeNode> node,
  std::shared_ptr<DomainExpert> & domain_expert_)
{
  if (!node) {
    return false;
  }

  switch (node->type_) {
    case parser::pddl::tree::AND: {
        std::shared_ptr<parser::pddl::tree::AndNode> pn_and =
          std::dynamic_pointer_cast<parser::pddl::tree::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          ret = ret && checkPredicateTreeTypes(op, domain_expert_);
        }
        return ret;
      }

    case parser::pddl::tree::OR: {
        std::shared_ptr<parser::pddl::tree::OrNode> pn_or =
          std::dynamic_pointer_cast<parser::pddl::tree::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          ret = ret && checkPredicateTreeTypes(op, domain_expert_);
        }
        return ret;
      }

    case parser::pddl::tree::NOT: {
        std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
          std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(node);

        return checkPredicateTreeTypes(pn_not->op, domain_expert_);
      }

    case parser::pddl::tree::PREDICATE: {
        std::shared_ptr<parser::pddl::tree::PredicateNode> pred =
          std::dynamic_pointer_cast<parser::pddl::tree::PredicateNode>(node);

        return isValidPredicate(pred->predicate_);
      }

    case parser::pddl::tree::FUNCTION: {
        std::shared_ptr<parser::pddl::tree::FunctionNode> func =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(node);

        return isValidFunction(func->function_);
      }

    case parser::pddl::tree::EXPRESSION: {
        std::shared_ptr<parser::pddl::tree::ExpressionNode> expression =
          std::dynamic_pointer_cast<parser::pddl::tree::ExpressionNode>(node);
        bool ret = true;

        for (const auto & op : expression->ops) {
          ret = ret && checkPredicateTreeTypes(op, domain_expert_);
        }
        return ret;
      }

    case parser::pddl::tree::FUNCTION_MODIFIER: {
        std::shared_ptr<parser::pddl::tree::FunctionModifierNode> fun_mod =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionModifierNode>(node);
        bool ret = true;

        for (const auto & op : fun_mod->ops) {
          ret = ret && checkPredicateTreeTypes(op, domain_expert_);
        }
        return ret;
      }

    case parser::pddl::tree::NUMBER: {
        return true;
      }

    default:
      // LCOV_EXCL_START
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return false;
}

std::string
ProblemExpert::getProblem()
{
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);

  problem.name = "problem_1";

  for (const parser::pddl::tree::Instance & instance : instances_) {
    problem.addObject(instance.name, instance.type);
  }

  for (parser::pddl::tree::Predicate predicate : predicates_) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addInit(predicate.name, v);
  }

  for (parser::pddl::tree::Function function : functions_) {
    StringVec v;

    for (size_t i = 0; i < function.parameters.size(); i++) {
      v.push_back(function.parameters[i].name);
    }

    std::transform(
      function.name.begin(), function.name.end(),
      function.name.begin(), ::tolower);

    problem.addInit(function.name, function.value, v);
  }

  std::vector<parser::pddl::tree::Predicate> predicates;
  goal_.getPredicates(predicates);

  for (auto predicate : predicates) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addGoal(predicate.name, v);
  }

  std::ostringstream stream;
  stream << problem;
  return stream.str();
}

}  // namespace plansys2
