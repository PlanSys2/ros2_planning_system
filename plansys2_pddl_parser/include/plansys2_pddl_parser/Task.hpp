// Copyright 2024 Intelligent Robotics Lab
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
#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/ParamCond.hpp"

namespace parser
{
namespace pddl
{

class Task : public ParamCond
{
public:
  Task() {}

  explicit Task(const std::string & s)
  : ParamCond(s) {}

  explicit Task(const ParamCond * c)
  : ParamCond(c) {}

  void PDDLPrint(
    std::ostream &, unsigned, const TokenStruct<std::string> &,
    const Domain &) const override
  {
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree &, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    throw UnsupportedConstruct("Task");
  }

  void parse(Stringreader &, TokenStruct<std::string> &, Domain &) {}

  void addParams(int, unsigned) {}

  Condition * copy(Domain &) {return new Task(this);}
};

typedef std::vector<Task *> TaskVec;

}  // namespace pddl
}  // namespace parser
