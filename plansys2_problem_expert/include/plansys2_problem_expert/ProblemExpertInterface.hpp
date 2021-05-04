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

#include "plansys2_core/Types.hpp"

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

class ProblemExpertInterface
{
public:
  ProblemExpertInterface() {}

  virtual std::vector<plansys2_msgs::msg::Param> getInstances() = 0;
  virtual bool addInstance(const plansys2::Instance & instance) = 0;
  virtual bool removeInstance(const plansys2::Instance & instance) = 0;
  virtual std::optional<plansys2_msgs::msg::Param> getInstance(const std::string & name) = 0;

  virtual std::vector<plansys2_msgs::msg::Node> getPredicates() = 0;
  virtual bool addPredicate(const plansys2_msgs::msg::Node & predicate) = 0;
  virtual bool removePredicate(const plansys2_msgs::msg::Node & predicate) = 0;
  virtual bool existPredicate(const plansys2_msgs::msg::Node & predicate) = 0;
  virtual std::optional<plansys2_msgs::msg::Node> getPredicate(const std::string & expr) = 0;

  virtual std::vector<plansys2_msgs::msg::Node> getFunctions() = 0;
  virtual bool addFunction(const plansys2_msgs::msg::Node & function) = 0;
  virtual bool removeFunction(const plansys2_msgs::msg::Node & function) = 0;
  virtual bool existFunction(const plansys2_msgs::msg::Node & function) = 0;
  virtual bool updateFunction(const plansys2_msgs::msg::Node & function) = 0;
  virtual std::optional<plansys2_msgs::msg::Node> getFunction(const std::string & expr) = 0;

  virtual plansys2_msgs::msg::Tree getGoal() = 0;
  virtual bool setGoal(const plansys2_msgs::msg::Tree & goal) = 0;
  virtual bool clearGoal() = 0;
  virtual bool clearKnowledge() = 0;
  virtual bool isGoalSatisfied(const plansys2_msgs::msg::Tree & goal) = 0;

  virtual std::string getProblem() = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
