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

namespace plansys2
{

ProblemExpert::ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert)
: domain_expert_(domain_expert)
{
}

bool
ProblemExpert::addInstance(const Instance & instance)
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

std::vector<Instance>
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
  removeAssignmentsReferencing(name);
  removePredicatesReferencing(name);

  return found;
}

boost::optional<Instance>
ProblemExpert::getInstance(const std::string & instance_name)
{
  Instance ret;

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

std::vector<Predicate>
ProblemExpert::getPredicates()
{
  return predicates_;
}

bool
ProblemExpert::addAssignment(const Assignment & assignment)
{
  if (!existAssignment(assignment)) {
    if (isValidAssignment(assignment)) {
      assignments_.push_back(assignment);
      return true;
    } else {
      return false;
    }
  } else {
    return updateAssignment(assignment);
  }
}

bool
ProblemExpert::addPredicate(const Predicate & predicate)
{
  if (!existPredicate(predicate)) {
    if (isValidPredicate(predicate)) {
      predicates_.push_back(predicate);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool
ProblemExpert::removePredicate(const Predicate & predicate)
{
  bool found = false;
  int i = 0;

  while (!found && i < predicates_.size()) {
    if (predicates_[i] == predicate) {
      found = true;
      predicates_.erase(predicates_.begin() + i);
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::removeAssignment(const Assignment & assignment)
{
  bool found = false;
  int i = 0;

  while (!found && i < assignments_.size()) {
    Assignment assignment = assignments_[i];
    if (assignments_[i].hasSameNamesAndParameters(assignment)) {
      found = true;
      assignments_.erase(assignments_.begin() + i);
    }
    i++;
  }

  return found;
}


bool
ProblemExpert::removeAssignmentsReferencing(const std::string & name)
{
  int i = 0;

  while (i < assignments_.size()) {
    bool found = false;
    for (Param parameter : assignments_[i].parameters) {
      if (parameter.name == name) {
        assignments_.erase(assignments_.begin() + i);
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
    for (Param parameter : predicates_[i].parameters) {
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

Goal
ProblemExpert::getGoal()
{
  return goal_;
}

bool
ProblemExpert::setGoal(const Goal & goal)
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
ProblemExpert::existAssignment(const Assignment & assignment)
{
  bool found = false;
  int i = 0;

  while (!found && i < assignments_.size()) {
    if (assignments_[i].name == assignment.name &&
      assignments_[i].parameters == assignment.parameters)
    {
      found = true;
    }
    i++;
  }

  return found;
}


bool
ProblemExpert::updateAssignment(const Assignment & assignment)
{
  bool found = false;
  int i = 0;

  while (!found && i < assignments_.size()) {
    if (assignments_[i].name == assignment.name &&
      assignments_[i].parameters == assignment.parameters)
    {
      assignments_[i].value = assignment.value;  /** update only the value */
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::existPredicate(const Predicate & predicate)
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
ProblemExpert::isValidAssignment(const Assignment & assignment)
{
  bool valid = false;

  const boost::optional<plansys2::Function> & model_function = domain_expert_->getFunction(
    assignment.name);
  if (model_function) {
    if (model_function.value().parameters.size() == assignment.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < assignment.parameters.size()) {
        auto arg_type = getInstance(assignment.parameters[i].name);

        if (!arg_type) {
          same_types = false;
        } else if (arg_type.value().type != model_function.value().parameters[i].type) {
          same_types = false;
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool
ProblemExpert::isValidPredicate(const Predicate & predicate)
{
  bool valid = false;

  const auto & model_predicate = domain_expert_->getPredicate(predicate.name);
  if (model_predicate) {
    if (model_predicate.value().parameters.size() == predicate.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < predicate.parameters.size()) {
        auto arg_type = getInstance(predicate.parameters[i].name);

        if (!arg_type) {
          same_types = false;
        } else if (arg_type.value().type != model_predicate.value().parameters[i].type) {
          same_types = false;
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool
ProblemExpert::isValidGoal(const Goal & goal)
{
  return checkPredicateTreeTypes(goal.root_, domain_expert_);
}

bool
ProblemExpert::checkPredicateTreeTypes(
  std::shared_ptr<TreeNode> node,
  std::shared_ptr<DomainExpert> & domain_expert_)
{
  switch (node->type_) {
    case AND: {
        std::shared_ptr<plansys2::AndNode> pn_and =
          std::dynamic_pointer_cast<plansys2::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          ret = ret && checkPredicateTreeTypes(op, domain_expert_);
        }
        return ret;
      }

    case OR: {
        std::shared_ptr<plansys2::OrNode> pn_or =
          std::dynamic_pointer_cast<plansys2::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          ret = ret && checkPredicateTreeTypes(op, domain_expert_);
        }
        return ret;
      }

    case NOT: {
        std::shared_ptr<plansys2::NotNode> pn_not =
          std::dynamic_pointer_cast<NotNode>(node);

        return checkPredicateTreeTypes(pn_not->op, domain_expert_);
      }

    case PREDICATE: {
        std::shared_ptr<plansys2::PredicateNode> pred =
          std::dynamic_pointer_cast<PredicateNode>(node);

        return isValidPredicate(pred->predicate_);
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }

  return false;
}

std::string
ProblemExpert::getProblem()
{
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);

  problem.name = "problem_1";

  for (const Instance & instance : instances_) {
    problem.addObject(instance.name, instance.type);
  }

  for (Predicate predicate : predicates_) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addInit(predicate.name, v);
  }

  for (Assignment assignment : assignments_) {
    StringVec v;

    for (size_t i = 0; i < assignment.parameters.size(); i++) {
      v.push_back(assignment.parameters[i].name);
    }

    std::transform(
      assignment.name.begin(), assignment.name.end(),
      assignment.name.begin(), ::tolower);

    problem.addInit(assignment.name, assignment.value, v);
  }

  std::vector<Predicate> predicates;
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
