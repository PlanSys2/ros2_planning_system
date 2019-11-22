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

DomainExpert::DomainExpert(const std::string & domain_file)
: domain_(domain_file)
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

std::optional<std::vector<std::string>>
DomainExpert::getPredicateParams(const std::string & predicate)
{
  std::string predicate_search = predicate;
  std::transform(predicate_search.begin(), predicate_search.end(),
    predicate_search.begin(), ::toupper);

  std::vector<std::string> ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.preds.size() && !found) {
    if (domain_.preds[i]->name == predicate_search) {
      found = true;
      for (unsigned j = 0; j < domain_.preds[i]->params.size(); j++) {
        ret.push_back(domain_.types[domain_.preds[i]->params[j]]->getName());
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
    ret.push_back(domain_.actions[i]->name);
  }
  return ret;
}

std::optional<std::vector<std::string>>
DomainExpert::getActionParams(const std::string & action)
{
  std::string action_search = action;
  std::transform(action_search.begin(), action_search.end(),
    action_search.begin(), ::toupper);

  std::vector<std::string> ret;
  bool found = false;
  unsigned i = 0;

  while (i < domain_.actions.size() && !found) {
    if (domain_.actions[i]->name == action_search) {
      found = true;
      for (unsigned j = 0; j < domain_.actions[i]->params.size(); j++) {
        ret.push_back(domain_.types[domain_.actions[i]->params[j]]->getName());
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
