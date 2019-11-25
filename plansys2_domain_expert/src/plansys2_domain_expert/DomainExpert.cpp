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


namespace plansys2
{

DomainExpert::DomainExpert(const std::string & domain)
: domain_(domain)
{
}

std::vector<std::string>
DomainExpert::getTypes()
{
  std::vector<std::string> ret;
  if (domain_.typed) {
    for (unsigned i = 1; i < domain_.types.size(); i++) {
      ret.push_back(domain_.types[i]->name);
    }
  }
  return ret;
}

std::vector<std::string>
DomainExpert::getPredicates()
{
  std::vector<std::string> ret;
  for (unsigned i = 0; i < domain_.preds.size(); i++) {
    ret.push_back(domain_.preds[i]->name);
  }
  return ret;
}

std::optional<plansys2::Predicate>
DomainExpert::getPredicate(const std::string & predicate)
{
  std::string predicate_search = predicate;
  std::transform(predicate_search.begin(), predicate_search.end(),
    predicate_search.begin(), ::toupper);

  plansys2::Predicate ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.preds.size() && !found) {
    if (domain_.preds[i]->name == predicate_search) {
      found = true;
      ret.name = predicate_search;
      for (unsigned j = 0; j < domain_.preds[i]->params.size(); j++) {
        plansys2::Param param;
        param.name = "?" + domain_.types[domain_.preds[i]->params[j]]->getName() +
          std::to_string(j);
        param.type = domain_.types[domain_.preds[i]->params[j]]->name;
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
  for (unsigned i = 0; i < domain_.actions.size(); i++) {
    parser::pddl::Action * action_obj = dynamic_cast<parser::pddl::Action *>(domain_.actions[i]);
    if (action_obj != nullptr) {
      ret.push_back(domain_.actions[i]->name);
    }
  }
  return ret;
}

std::optional<plansys2::Action>
DomainExpert::getAction(const std::string & action)
{
  std::string action_search = action;
  std::transform(action_search.begin(), action_search.end(),
    action_search.begin(), ::toupper);

  plansys2::Action ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.actions.size() && !found) {
    parser::pddl::Action * action_obj = dynamic_cast<parser::pddl::Action *>(domain_.actions[i]);

    if (action_obj != nullptr && action_obj->name == action_search) {
      found = true;
      ret.name = action;

      // Parameters
      for (unsigned j = 0; j < action_obj->params.size(); j++) {
        Param param;
        param.name = "?" + domain_.types[action_obj->params[i]]->getName() + std::to_string(j);
        param.name = domain_.types[action_obj->params[i]]->name;
        ret.parameters.push_back(param);
      }

      // Preconditions
      if (action_obj->pre) {
        std::stringstream pre_stream;
        action_obj->pre->PDDLPrint(pre_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        std::cout << pre_stream.str() << std::endl;
        // ret.preconditions.fromString(pre_stream.str());
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
  for (unsigned i = 0; i < domain_.actions.size(); i++) {
    parser::pddl::TemporalAction * action_obj =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]);
    if (action_obj != nullptr) {
      ret.push_back(domain_.actions[i]->name);
    }
  }
  return ret;
}

std::optional<plansys2::DurativeAction>
DomainExpert::getDurativeAction(const std::string & action)
{
  std::string action_search = action;
  std::transform(action_search.begin(), action_search.end(),
    action_search.begin(), ::toupper);

  plansys2::DurativeAction ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.actions.size() && !found) {
    parser::pddl::TemporalAction * action_obj =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]);

    if (action_obj != nullptr && action_obj->name == action_search) {
      found = true;
      ret.name = action;

      // Parameters
      for (unsigned j = 0; j < action_obj->params.size(); j++) {
        Param param;
        param.name = "?" + domain_.types[action_obj->params[i]]->getName() + std::to_string(j);
        param.name = domain_.types[action_obj->params[i]]->name;
        ret.parameters.push_back(param);
      }

      // Preconditions
      if (action_obj->pre) {
        std::stringstream pre_stream;
        action_obj->pre->PDDLPrint(pre_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        std::cout << pre_stream.str() << std::endl;
        ret.at_end_requirements.fromString(pre_stream.str());
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
}  // namespace plansys2
