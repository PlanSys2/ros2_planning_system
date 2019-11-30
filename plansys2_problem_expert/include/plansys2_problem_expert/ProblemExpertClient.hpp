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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "pddl_parser/Instance.h"

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"

namespace plansys2
{

struct Instance
{
  std::string name;
  std::string type;
};

class ProblemExpert : public ProblemExpertInterface
{
public:
  ProblemExpert();

  void addInstance(std::string name, std::string type);
  std::vector<std::string> getInstances();
  std::optional<std::string> getInstanceType(const std::string & instance_name);

  /*std::optional<std::vector<std::string>> getPredicateParams(const std::string & predicate);
  std::vector<std::string> getActions();
  std::optional<std::vector<std::string>> getActionParams(const std::string & action);*/

  // const parser::pddl::Problem & getProblem() {return problem_;}

private:
  // parser::pddl::Problem problem_;
  std::vector<Instance> instances_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
