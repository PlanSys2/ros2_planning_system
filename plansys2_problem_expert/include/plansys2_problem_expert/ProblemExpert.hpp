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

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Utils.h"
#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

namespace plansys2
{

class ProblemExpert : public ProblemExpertInterface
{
public:
  explicit ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert);

  std::vector<plansys2_msgs::msg::Param> getInstances();
  bool addInstance(const plansys2_msgs::msg::Param & instance);
  bool removeInstance(const plansys2_msgs::msg::Param & instance);
  std::optional<plansys2_msgs::msg::Param> getInstance(const std::string & name);

  std::vector<plansys2_msgs::msg::Node> getPredicates();
  bool addPredicate(const plansys2_msgs::msg::Node & predicate);
  bool removePredicate(const plansys2_msgs::msg::Node & predicate);
  std::optional<plansys2_msgs::msg::Node> getPredicate(const std::string & expr);

  std::vector<plansys2_msgs::msg::Node> getFunctions();
  bool addFunction(const plansys2_msgs::msg::Node & function);
  bool removeFunction(const plansys2_msgs::msg::Node & function);
  bool updateFunction(const plansys2_msgs::msg::Node & function);
  std::optional<plansys2_msgs::msg::Node> getFunction(const std::string & expr);

  plansys2_msgs::msg::Tree getGoal();
  bool setGoal(const plansys2_msgs::msg::Tree & goal);
  bool clearGoal();
  bool clearKnowledge();
  bool isGoalSatisfied(const plansys2_msgs::msg::Tree & goal);

  std::string getProblem();

  bool existInstance(const std::string & name);
  bool existPredicate(const plansys2_msgs::msg::Node & predicate);
  bool existFunction(const plansys2_msgs::msg::Node & function);
  bool isValidType(const std::string & type);
  bool isValidPredicate(const plansys2_msgs::msg::Node & predicate);
  bool isValidFunction(const plansys2_msgs::msg::Node & function);
  bool isValidGoal(const plansys2_msgs::msg::Tree & goal);

private:
  bool checkPredicateTreeTypes(
    const plansys2_msgs::msg::Tree & tree,
    std::shared_ptr<DomainExpert> & domain_expert_,
    uint8_t node_id = 0);

  bool removeFunctionsReferencing(const plansys2_msgs::msg::Param & param);
  bool removePredicatesReferencing(const plansys2_msgs::msg::Param & param);

  std::vector<plansys2_msgs::msg::Param> instances_;
  std::vector<plansys2_msgs::msg::Node> predicates_;
  std::vector<plansys2_msgs::msg::Node> functions_;
  plansys2_msgs::msg::Tree goal_;

  std::shared_ptr<DomainExpert> domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
