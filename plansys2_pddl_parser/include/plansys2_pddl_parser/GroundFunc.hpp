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

#include <string>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/TypeGround.hpp"

namespace parser
{
namespace pddl
{

template<typename T>
class GroundFunc : public TypeGround
{
public:
  T value;

  GroundFunc()
  : TypeGround(), value(0) {}

  explicit GroundFunc(Lifted * l, const T & val = T(0))
  : TypeGround(l), value(val) {}

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  void print(std::ostream & stream) const {stream << name << params << value << "\n";}

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);
};

}  // namespace pddl
}  // namespace parser
