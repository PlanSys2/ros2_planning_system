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

#include "pddl_parser/Instance.h"

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

#include "plansys2_domain_expert/Types.hpp"
#include "plansys2_problem_expert/Types.hpp"

namespace plansys2
{

class ProblemExpert : public ProblemExpertInterface
{
public:
  explicit ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert);

  std::vector<Instance> getInstances();
  bool addInstance(const Instance & instance);
  bool removeInstance(const std::string & name);
  std::optional<Instance> getInstance(const std::string & name);

  std::vector<Predicate> getPredicates();
  bool addPredicate(const Predicate & predicate);
  bool removePredicate(const Predicate & predicate);

  Goal getGoal();
  bool setGoal(const Goal & goal);
  bool clearGoal();

  std::string getProblem();

  bool existInstance(const std::string & name);
  bool existPredicate(const Predicate & predicate);
  bool existGoal(const Goal & goal);
  bool isValidType(const std::string & type);
  bool isValidPredicate(const Predicate & predicate);
  bool isValidGoal(const Goal & goal);

private:
  bool checkPredicateTreeTypes(
    std::shared_ptr<TreeNode> node,
    std::shared_ptr<DomainExpert> & domain_expert_);

  // parser::pddl::Problem problem_;
  std::vector<Instance> instances_;
  std::vector<Predicate> predicates_;
  Goal goal_;

  std::shared_ptr<DomainExpert> domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
