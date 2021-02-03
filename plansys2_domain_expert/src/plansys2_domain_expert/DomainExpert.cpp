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

#include "plansys2_domain_expert/DomainExpert.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>


namespace plansys2
{

DomainExpert::DomainExpert(const std::string & domain)
{
  extendDomain(domain);
}

void
DomainExpert::extendDomain(const std::string & domain)
{
  domains_.add_domain(domain);

  domain_ = std::make_shared<parser::pddl::Domain>();

  try {
    domain_->parse(domains_.get_joint_domain());
  } catch (const std::exception & e) {
    std::cerr << "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError parsing PDDL: " << e.what() << std::endl;
    std::cerr << "Error parsing PDDL: " << e.what() << std::endl;
  }
}

std::vector<std::string>
DomainExpert::getTypes()
{
  std::vector<std::string> ret;
  if (domain_->typed) {
    for (unsigned i = 1; i < domain_->types.size(); i++) {
      ret.push_back(domain_->types[i]->name);
    }
  }
  return ret;
}

std::vector<std::string>
DomainExpert::getPredicates()
{
  std::vector<std::string> ret;
  for (unsigned i = 0; i < domain_->preds.size(); i++) {
    ret.push_back(domain_->preds[i]->name);
  }
  return ret;
}

std::optional<parser::pddl::tree::Predicate>
DomainExpert::getPredicate(const std::string & predicate)
{
  std::string predicate_search = predicate;
  std::transform(
    predicate_search.begin(), predicate_search.end(),
    predicate_search.begin(), ::tolower);

  parser::pddl::tree::Predicate ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_->preds.size() && !found) {
    if (domain_->preds[i]->name == predicate_search) {
      found = true;
      ret.name = predicate_search;
      for (unsigned j = 0; j < domain_->preds[i]->params.size(); j++) {
        parser::pddl::tree::Param param;
        param.name = "?" + domain_->types[domain_->preds[i]->params[j]]->getName() +
          std::to_string(j);
        param.type = domain_->types[domain_->preds[i]->params[j]]->name;
        domain_->types[domain_->preds[i]->params[j]]->getSubTypesNames(param.subTypes);
        ret.parameters.push_back(param);
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

std::vector<std::string>
DomainExpert::getFunctions()
{
  std::vector<std::string> ret;
  for (unsigned i = 0; i < domain_->funcs.size(); i++) {
    ret.push_back(domain_->funcs[i]->name);
  }
  return ret;
}

std::optional<parser::pddl::tree::Function>
DomainExpert::getFunction(const std::string & function)
{
  std::string function_search = function;
  std::transform(
    function_search.begin(), function_search.end(),
    function_search.begin(), ::tolower);

  parser::pddl::tree::Function ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_->funcs.size() && !found) {
    if (domain_->funcs[i]->name == function_search) {
      found = true;
      ret.name = function_search;
      for (unsigned j = 0; j < domain_->funcs[i]->params.size(); j++) {
        parser::pddl::tree::Param param;
        param.name = "?" + domain_->types[domain_->funcs[i]->params[j]]->getName() +
          std::to_string(j);
        param.type = domain_->types[domain_->funcs[i]->params[j]]->name;
        domain_->types[domain_->funcs[i]->params[j]]->getSubTypesNames(param.subTypes);
        ret.parameters.push_back(param);
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

std::vector<std::string>
DomainExpert::getActions()
{
  std::vector<std::string> ret;
  for (unsigned i = 0; i < domain_->actions.size(); i++) {
    bool is_action = dynamic_cast<parser::pddl::TemporalAction *>(domain_->actions[i]) == nullptr;
    parser::pddl::Action * action_obj = dynamic_cast<parser::pddl::Action *>(domain_->actions[i]);
    if (is_action) {
      ret.push_back(domain_->actions[i]->name);
    }
  }
  return ret;
}

std::optional<parser::pddl::tree::Action>
DomainExpert::getAction(const std::string & action)
{
  std::string action_search = action;
  std::transform(
    action_search.begin(), action_search.end(),
    action_search.begin(), ::tolower);

  parser::pddl::tree::Action ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_->actions.size() && !found) {
    bool is_action = dynamic_cast<parser::pddl::TemporalAction *>(domain_->actions[i]) == nullptr;
    parser::pddl::Action * action_obj = dynamic_cast<parser::pddl::Action *>(domain_->actions[i]);

    if (is_action && action_obj->name == action_search) {
      found = true;
      ret.name = action;

      // Parameters
      for (unsigned j = 0; j < action_obj->params.size(); j++) {
        parser::pddl::tree::Param param;
        param.name = "?" + std::to_string(j);
        param.type = domain_->types[action_obj->params[j]]->name;
        domain_->types[action_obj->params[j]]->getSubTypesNames(param.subTypes);
        ret.parameters.push_back(param);
      }

      // Preconditions
      if (action_obj->pre) {
        ret.preconditions.root_ = action_obj->pre->PDDLTree(*domain_);
      }

      // Effects
      if (action_obj->eff) {
        ret.effects.root_ = action_obj->eff->PDDLTree(*domain_);
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

std::vector<std::string>
DomainExpert::getDurativeActions()
{
  std::vector<std::string> ret;
  for (unsigned i = 0; i < domain_->actions.size(); i++) {
    bool is_durative_action =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_->actions[i]) != nullptr;
    parser::pddl::TemporalAction * action_obj =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_->actions[i]);
    if (is_durative_action) {
      ret.push_back(domain_->actions[i]->name);
    }
  }
  return ret;
}

std::optional<parser::pddl::tree::DurativeAction>
DomainExpert::getDurativeAction(const std::string & action)
{
  std::string action_search = action;
  std::transform(
    action_search.begin(), action_search.end(),
    action_search.begin(), ::tolower);

  parser::pddl::tree::DurativeAction ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_->actions.size() && !found) {
    bool is_durative_action =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_->actions[i]) != nullptr;
    parser::pddl::TemporalAction * action_obj =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_->actions[i]);

    if (is_durative_action && action_obj->name == action_search) {
      found = true;
      ret.name = action;

      // Parameters
      for (unsigned j = 0; j < action_obj->params.size(); j++) {
        parser::pddl::tree::Param param;
        param.name = "?" + std::to_string(j);
        param.type = domain_->types[action_obj->params[j]]->name;
        domain_->types[action_obj->params[j]]->getSubTypesNames(param.subTypes);
        ret.parameters.push_back(param);
      }

      // Preconditions AtStart
      if (action_obj->pre) {
        ret.at_start_requirements.root_ = action_obj->pre->PDDLTree(*domain_);
      }

      // Preconditions OverAll
      if (action_obj->pre_o) {
        ret.over_all_requirements.root_ = action_obj->pre_o->PDDLTree(*domain_);
      }

      // Preconditions AtEnd
      if (action_obj->pre_e) {
        ret.at_end_requirements.root_ = action_obj->pre_e->PDDLTree(*domain_);
      }

      // Effects AtStart
      if (action_obj->eff) {
        ret.at_start_effects.root_ = action_obj->eff->PDDLTree(*domain_);
      }

      // Effects AtEnd
      if (action_obj->eff_e) {
        ret.at_end_effects.root_ = action_obj->eff_e->PDDLTree(*domain_);
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

std::string
DomainExpert::getDomain()
{
  return domains_.get_joint_domain();
}

}  // namespace plansys2
