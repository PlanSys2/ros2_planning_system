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
#include <stdexcept>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Basic.hpp"
#include "plansys2_pddl_parser/Stringreader.hpp"
#include "plansys2_pddl_parser/Type.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace parser
{
namespace pddl
{

class UnsupportedConstruct : public std::runtime_error
{
public:
  explicit UnsupportedConstruct(const std::string & construct)
  : std::runtime_error(construct + " is not currently supported by plansys2")
  {
  }
};

class Condition
{
public:
  virtual ~Condition() {}

  virtual void print(std::ostream & stream) const = 0;

  virtual void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const = 0;

  virtual plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const = 0;

  virtual void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d) = 0;

  virtual void addParams(int m, unsigned n) = 0;

  virtual Condition * copy(Domain & d) = 0;
};

inline std::ostream & operator<<(std::ostream & stream, const Condition * c)
{
  c->print(stream);
  return stream;
}

typedef std::vector<Condition *> CondVec;

}  // namespace pddl
}  // namespace parser
