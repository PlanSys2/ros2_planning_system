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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_

#include <string>
#include <vector>

namespace plansys2
{

class ProblemExpertInterface
{
public:
  ProblemExpertInterface() {}

  virtual std::vector<std::string> getInstances() = 0;
  virtual bool addInstance(const std::string & name, const std::string & type) = 0;
  virtual bool removeInstance(const std::string & name) = 0;
  virtual std::optional<std::string> getInstanceType(const std::string & name) = 0;

  virtual std::vector<std::string> getPredicates() = 0;
  virtual bool addPredicate(const std::string & predicate, const std::vector<std::string> & arguments) = 0;
  virtual bool removePredicate(const std::string & predicate, const std::vector<std::string> & arguments) = 0;
  virtual std::optional<std::vector<std::string>> getPredicateArguments(const std::string & predicate) = 0;

  virtual std::vector<std::string> getGoal() = 0;
  virtual bool addGoal(const std::string & goal, const std::vector<std::string> & arguments) = 0;
  virtual bool removeGoal(const std::string & goal, const std::vector<std::string> & arguments) = 0;

  virtual std::string getProblem() = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
