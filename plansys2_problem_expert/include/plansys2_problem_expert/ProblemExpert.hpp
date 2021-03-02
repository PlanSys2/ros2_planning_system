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
#include <memory>

#include "plansys2_pddl_parser/Instance.h"
#include "plansys2_pddl_parser/Tree.h"

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

namespace plansys2
{

class ProblemExpert : public ProblemExpertInterface
{
public:
  explicit ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert);

  std::vector<parser::pddl::tree::Instance> getInstances();
  bool addInstance(const parser::pddl::tree::Instance & instance);
  bool removeInstance(const std::string & name);
  std::optional<parser::pddl::tree::Instance> getInstance(const std::string & name);

  std::vector<parser::pddl::tree::Predicate> getPredicates();
  bool addPredicate(const parser::pddl::tree::Predicate & predicate);
  bool removePredicate(const parser::pddl::tree::Predicate & predicate);
  std::optional<parser::pddl::tree::Predicate> getPredicate(const std::string & expr);

  std::vector<parser::pddl::tree::Function> getFunctions();
  bool addFunction(const parser::pddl::tree::Function & function);
  bool removeFunction(const parser::pddl::tree::Function & function);
  bool updateFunction(const parser::pddl::tree::Function & function);
  std::optional<parser::pddl::tree::Function> getFunction(const std::string & expr);

  parser::pddl::tree::Goal getGoal();
  bool setGoal(const parser::pddl::tree::Goal & goal);
  bool clearGoal();
  bool clearKnowledge();
  bool isGoalSatisfied(const parser::pddl::tree::Goal & goal);

  std::string getProblem();

  bool existInstance(const std::string & name);
  bool existPredicate(const parser::pddl::tree::Predicate & predicate);
  bool existFunction(const parser::pddl::tree::Function & function);
  bool existGoal(const parser::pddl::tree::Goal & goal);
  bool isValidType(const std::string & type);
  bool isValidPredicate(const parser::pddl::tree::Predicate & predicate);
  bool isValidFunction(const parser::pddl::tree::Function & function);
  bool isValidGoal(const parser::pddl::tree::Goal & goal);

private:
  bool checkPredicateTreeTypes(
    std::shared_ptr<parser::pddl::tree::TreeNode> node,
    std::shared_ptr<DomainExpert> & domain_expert_);

  bool removeFunctionsReferencing(const std::string & name);
  bool removePredicatesReferencing(const std::string & name);

  // parser::pddl::Problem problem_;
  std::vector<parser::pddl::tree::Instance> instances_;
  std::vector<parser::pddl::tree::Predicate> predicates_;
  std::vector<parser::pddl::tree::Function> functions_;
  parser::pddl::tree::Goal goal_;

  std::shared_ptr<DomainExpert> domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
