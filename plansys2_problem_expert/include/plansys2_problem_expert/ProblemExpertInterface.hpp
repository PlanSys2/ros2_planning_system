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

#include "plansys2_pddl_parser/Tree.h"

namespace plansys2
{

class ProblemExpertInterface
{
public:
  ProblemExpertInterface() {}

  virtual std::vector<parser::pddl::tree::Instance> getInstances() = 0;
  virtual bool addInstance(const parser::pddl::tree::Instance & instance) = 0;
  virtual bool removeInstance(const std::string & name) = 0;
  virtual std::optional<parser::pddl::tree::Instance> getInstance(const std::string & name) = 0;

  virtual std::vector<parser::pddl::tree::Predicate> getPredicates() = 0;
  virtual bool addPredicate(const parser::pddl::tree::Predicate & predicate) = 0;
  virtual bool removePredicate(const parser::pddl::tree::Predicate & predicate) = 0;
  virtual bool existPredicate(const parser::pddl::tree::Predicate & predicate) = 0;
  virtual std::optional<parser::pddl::tree::Predicate> getPredicate(const std::string & expr) = 0;

  virtual std::vector<parser::pddl::tree::Function> getFunctions() = 0;
  virtual bool addFunction(const parser::pddl::tree::Function & function) = 0;
  virtual bool removeFunction(const parser::pddl::tree::Function & function) = 0;
  virtual bool existFunction(const parser::pddl::tree::Function & function) = 0;
  virtual bool updateFunction(const parser::pddl::tree::Function & function) = 0;
  virtual std::optional<parser::pddl::tree::Function> getFunction(const std::string & expr) = 0;

  virtual parser::pddl::tree::Goal getGoal() = 0;
  virtual bool setGoal(const parser::pddl::tree::Goal & goal) = 0;
  virtual bool clearGoal() = 0;
  virtual bool clearKnowledge() = 0;
  virtual bool isGoalSatisfied(const parser::pddl::tree::Goal & goal) = 0;

  virtual std::string getProblem() = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
