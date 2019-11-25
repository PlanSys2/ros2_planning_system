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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "pddl_parser/Instance.h"

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

namespace plansys2
{

struct Instance
{
  std::string name;
  std::string type;
};

struct Predicate
{
  std::string predicate;
  std::vector<std::string> arguments;
};

struct Goal
{
  std::string goal;
  std::vector<std::string> arguments;
};

class ProblemExpert : public ProblemExpertInterface
{
public:
  ProblemExpert();

  std::vector<std::string> getInstances();
  bool addInstance(const std::string & name, const std::string & type);
  bool removeInstance(const std::string & name);
  std::optional<std::string> getInstanceType(const std::string & name);

  std::vector<std::string> getPredicates();
  bool addPredicate(const std::string & predicate, const std::vector<std::string> & arguments);
  bool removePredicate(const std::string & predicate, const std::vector<std::string> & arguments);
  std::optional<std::vector<std::string>> getPredicateArguments(const std::string & predicate);

  std::vector<std::string> getGoal();
  bool addGoal(const std::string & goal, const std::vector<std::string> & arguments);
  bool removeGoal(const std::string & goal, const std::vector<std::string> & arguments);

  std::string getProblem();

  bool validType(const std::string & type);
  bool existInstance(const std::string & name);
  bool existPredicate(const std::string & predicate, const std::vector<std::string> & arguments);
  bool validPredicate(const std::string & predicate);
  
private:
  // parser::pddl::Problem problem_;
  std::vector<Instance> instances_;
  std::vector<Predicate> predicates_;
  std::vector<Goal> goals_;

  plansys2::DomainExpert domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
