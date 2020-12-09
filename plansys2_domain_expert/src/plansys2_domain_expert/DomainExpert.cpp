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

void
DomainExpert::extendDomain(const std::string & domain)
{
  domain_.parse(domain);
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
DomainExpert::getFunctions() {
  std::vector<std::string> ret;
  for (unsigned i = 0; i < domain_.funcs.size(); i++) {
    ret.push_back(domain_.funcs[i]->name);
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

/**
 * @brief Search a function in the Domain and return it.
 * 
 * @param function name of the function
 * @return boost::optional<plansys2::Function> 
 *  The parameters name is the type name, prefixed by '?' 
 *  and suffixed by the parameter index (starting at 0).
 */
boost::optional<plansys2::Function> DomainExpert::getFunction(const std::string & function) {
  std::string function_search = function;
  std::transform(
    function_search.begin(), function_search.end(),
    function_search.begin(), ::tolower);
  plansys2::Function ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.funcs.size() && !found) {
    if (domain_.funcs[i]->name == function_search) {
      found = true;
      ret.name = function_search;
      for (unsigned j = 0; j < domain_.funcs[i]->params.size(); j++) {
        plansys2::Param param;
        param.name = "?" + domain_.types[domain_.funcs[i]->params[j]]->getName() + std::to_string(j);
        param.type =       domain_.types[domain_.funcs[i]->params[j]]->getName();
        domain_.types[domain_.funcs[i]->params[j]]->getSubTypesNames(param.subTypes);
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


boost::optional<plansys2::Predicate>
DomainExpert::getPredicate(const std::string & predicate)
{
  std::string predicate_search = predicate;
  std::transform(
    predicate_search.begin(), predicate_search.end(),
    predicate_search.begin(), ::tolower);

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
    bool is_action = dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]) == nullptr;
    parser::pddl::Action * action_obj = dynamic_cast<parser::pddl::Action *>(domain_.actions[i]);
    if (is_action) {
      ret.push_back(domain_.actions[i]->name);
    }
  }
  return ret;
}

boost::optional<plansys2::Action>
DomainExpert::getAction(const std::string & action)
{
  std::string action_search = action;
  std::transform(
    action_search.begin(), action_search.end(),
    action_search.begin(), ::tolower);

  plansys2::Action ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.actions.size() && !found) {
    bool is_action = dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]) == nullptr;
    parser::pddl::Action * action_obj = dynamic_cast<parser::pddl::Action *>(domain_.actions[i]);

    if (is_action && action_obj->name == action_search) {
      found = true;
      ret.name = action;

      // Parameters
      for (unsigned j = 0; j < action_obj->params.size(); j++) {
        Param param;
        param.name = "?" + std::to_string(j);
        param.type = domain_.types[action_obj->params[j]]->name;
        ret.parameters.push_back(param);
      }

      // Preconditions
      if (action_obj->pre) {
        std::stringstream pre_stream;
        action_obj->pre->PDDLPrint(
          pre_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        ret.preconditions.fromString(pre_stream.str());
      }

      // Effects
      if (action_obj->eff) {
        std::stringstream effects_stream;
        action_obj->eff->PDDLPrint(
          effects_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        ret.effects.fromString(effects_stream.str());
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
    bool is_durative_action =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]) != nullptr;
    parser::pddl::TemporalAction * action_obj =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]);
    if (is_durative_action) {
      ret.push_back(domain_.actions[i]->name);
    }
  }
  return ret;
}

boost::optional<plansys2::DurativeAction>
DomainExpert::getDurativeAction(const std::string & action)
{
  std::string action_search = action;
  std::transform(
    action_search.begin(), action_search.end(),
    action_search.begin(), ::tolower);

  plansys2::DurativeAction ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.actions.size() && !found) {
    bool is_durative_action =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]) != nullptr;
    parser::pddl::TemporalAction * action_obj =
      dynamic_cast<parser::pddl::TemporalAction *>(domain_.actions[i]);

    if (is_durative_action && action_obj->name == action_search) {
      found = true;
      ret.name = action;

      // Parameters
      for (unsigned j = 0; j < action_obj->params.size(); j++) {
        Param param;
        param.name = "?" + std::to_string(j);
        param.type = domain_.types[action_obj->params[j]]->name;
        ret.parameters.push_back(param);
      }

      // Preconditions AtStart
      if (action_obj->pre) {
        {
          std::stringstream pre_stream;
          action_obj->pre->PDDLPrint(
            pre_stream, 0,
            parser::pddl::TokenStruct<std::string>(), domain_);
          ret.at_start_requirements.fromString(pre_stream.str());
        }
      }

      // Preconditions OverAll
      if (action_obj->pre_o) {
        std::stringstream pre_stream;
        action_obj->pre_o->PDDLPrint(
          pre_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        ret.over_all_requirements.fromString(pre_stream.str());
      }

      // Preconditions AtEnd
      if (action_obj->pre_e) {
        std::stringstream pre_stream;
        action_obj->pre_e->PDDLPrint(
          pre_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        ret.at_end_requirements.fromString(pre_stream.str());
      }

      // Effects AtStart
      if (action_obj->eff) {
        std::stringstream effects_stream;
        action_obj->eff->PDDLPrint(
          effects_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        ret.at_start_effects.fromString(effects_stream.str());
      }

      // Effects AtEnd
      if (action_obj->eff_e) {
        std::stringstream effects_stream;
        action_obj->eff_e->PDDLPrint(
          effects_stream, 0,
          parser::pddl::TokenStruct<std::string>(), domain_);

        ret.at_end_effects.fromString(effects_stream.str());
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
  std::ostringstream stream;
  stream << domain_;
  return stream.str();
}

}  // namespace plansys2
