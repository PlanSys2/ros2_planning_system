// Copyright 2026 Intelligent Robotics Lab
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

#ifndef PLANSYS2_PDDL_PARSER__CONSTRAINT_HPP_
#define PLANSYS2_PDDL_PARSER__CONSTRAINT_HPP_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Condition.hpp"

namespace parser
{
namespace pddl
{

// A PDDL3 modal constraint over the states of a plan: (always C), (within 10 C),
// (sometime-before C1 C2), (hold-during 2 5 C), (at end C)...
class Constraint : public Condition
{
public:
  std::string op;
  std::vector<double> times;
  CondVec conds;

  explicit Constraint(const std::string & o)
  : op(o) {}

  Constraint(const Constraint * c, Domain & d)
  : op(c->op), times(c->times)
  {
    for (auto * cond : c->conds) {
      conds.push_back(cond->copy(d));
    }
  }

  ~Constraint()
  {
    for (auto * cond : conds) {
      delete cond;
    }
  }

  // The operators, and how many times and conditions each one takes
  static bool isOperator(const std::string & o);
  static std::pair<unsigned, unsigned> arity(const std::string & o);

  void print(std::ostream & s) const
  {
    s << "Constraint " << op << ":\n";
    for (auto * cond : conds) {
      cond->print(s);
    }
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

  void addParams(int m, unsigned n)
  {
    for (auto * cond : conds) {
      cond->addParams(m, n);
    }
  }

  Condition * copy(Domain & d) {return new Constraint(this, d);}
};

}  // namespace pddl
}  // namespace parser

#endif  // PLANSYS2_PDDL_PARSER__CONSTRAINT_HPP_
