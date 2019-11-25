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


namespace plansys2
{

ProblemExpert::ProblemExpert()
{
}

bool 
ProblemExpert::addInstance(std::string name, std::string type)
{
  if (!validType(type))
  {
    std::cerr << "Trying to add an instance [" << name << " of unknown type " << type << std::endl; 
    return false;
  } else if (existInstance(name)) {
    std::cerr << "Trying to add an existing instance [" << name << " of type " << type << std::endl; 
    return false;
  } else {
    instances_.push_back(Instance{name, type});
    return true;
  }
}

std::vector<std::string>
ProblemExpert::getInstances()
{
  std::vector<std::string> ret;

  for (const auto & instance : instances_)
  {
    ret.push_back(instance.name);
  }

  return ret;
}


bool
ProblemExpert::removeInstance(const std::string & name)
{
  found = false;
  int i = 0;

  while (!found && i < instances_.size())
  {
    if (instances_[i] == name)
    {
      found = true;
      instances_.erase(instances_.begin() + i);
    }
  }
  
  return found;
}

std::optional<std::string>
ProblemExpert::getInstanceType(const std::string & instance_name)
{
  std::string ret;

  bool found = false;
  int i = 0;
  while (i < instances_.size() && !found) {
    if (instances_[i].name == instance_name) {
      found = true;
      ret = instances_[i].type;
    }
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<std::string>
ProblemExpert::getPredicates()
{
  std::vector<std::string> ret;

  for (auto const & predicate : predicates_)
  {
    ret.push_back(predicate.predicate);
  }

  return ret;
}

bool
ProblemExpert::addPredicate(const std::string & predicate, const std::vector<std::string> & arguments)
{
  if (!existPredicate(predicate, arguments) && validPredicate(predicate))
  {
    auto param_types = getPredicateArguments(predicate);
    if (!param_types.has_value()) {
      std::cerr << "Predicate to add does not exists [" << predicate << "]" << std::endl;
      return false;
    } else if (!param_types.value().size() != arguments.size()) {
      std::cerr << "Predicates params number [" << param_types.value().size() <<
        "]does not fit with args number [" << arguments.size() << "]" << std::endl;
      return false;
    }

    for (int i = 0; i < arguments.size(); i++)
    {
      auto instance_type = getInstanceType(arguments[i]);
      if (!instance_type.has_value()) {
        std::cerr << "Instance does not exists [" << arguments[i] <<
          "] adding predicate" << std::endl;
        return false;
      } else if (instance_type.value() != param_types.value()[i]) {
        std::cerr << "Instance [" << arguments[i] << "] type [" << instance_type.value() <<
          "] does not fit with predicate param [" << param_types.value()[i] << "]" << std::endl;
        return false;
      }
    }

    predicates_.push_back(Predicate{predicate, arguments});
    return true;
  } else {
    std::cerr << "Trying to add an existing predicate [" << name << " with args " << type << std::endl; 
    return false;
  }
}

void
ProblemExpert::removePredicate(const std::string & predicate, const std::vector<std::string> & arguments)
{

}

std::optional<std::vector<std::string>>
ProblemExpert::getPredicateArguments(const std::string & predicate)
{
  return {};
}

std::vector<std::string>
ProblemExpert::getGoal()
{
  std::vector<std::string> ret;

  return ret;
}

bool
ProblemExpert::addGoal(const std::string & goal, const std::vector<std::string> & arguments)
{
  return true;
}

bool
ProblemExpert::removeGoal(const std::string & goal, const std::vector<std::string> & arguments)
{
  return true;
}

std::string::addGoal
ProblemExpert::getProblem()
{
  return "";
}

bool
ProblemExpert::validType(const std::string & type)
{
  auto valid_types = domain_expert_.getTypes();
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
  }

  return found;
}

bool
ProblemExpert::existPredicate(const std::string & predicate, const std::vector<std::string> & arguments)
{
  bool found = false;
  int i = 0;

  while (!found && i < predicates_.size()) {
    if (predicates_[i].predicate == predicate && arguments == predicates_[i].arguments) {
      found = true;
    }
  }

  return found;
}

bool
ProblemExpert::validPredicate(const std::string & predicate)
{
  auto valid_predicates = domain_expert_.getPredicates();
  auto it = std::find(valid_predicates.begin(), valid_predicates.end(), predicate);

  return it != valid_predicates.end();
}


}  // namespace plansys2
