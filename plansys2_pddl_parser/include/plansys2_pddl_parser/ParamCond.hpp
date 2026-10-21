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

#include "plansys2_pddl_parser/Basic.hpp"
#include "plansys2_pddl_parser/Condition.hpp"

namespace parser
{
namespace pddl
{

// This is necessary for adl
using ::operator<<;

class ParamCond : public Condition
{
public:
  std::string name;
  IntVec params;

  ParamCond() {}

  explicit ParamCond(const std::string & s, const IntVec & p = IntVec())
  : name(s), params(p) {}

  explicit ParamCond(const ParamCond * c)
  : name(c->name), params(c->params) {}

  std::string c_str() const {return name;}

  void print(std::ostream & stream) const {stream << name << params << "\n";}

  void printParams(
    unsigned first, std::ostream & s, TokenStruct<std::string> & ts, const Domain & d) const;
};

typedef std::vector<ParamCond *> ParamCondVec;

}  // namespace pddl
}  // namespace parser
