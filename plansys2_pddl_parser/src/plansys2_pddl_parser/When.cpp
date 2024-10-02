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
#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

void When::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( when\n";
  if (pars) {
    pars->PDDLPrint(s, indent + 1, ts, d);
  } else {
    tabindent(s, indent + 1);
    s << "()";
  }
  s << "\n";
  if (cond) {
    cond->PDDLPrint(s, indent + 1, ts, d);
  } else {
    tabindent(s, indent + 1);
    s << "()";
  }
  s << "\n";
  tabindent(s, indent);
  s << ")";
}

plansys2_msgs::msg::Node::SharedPtr When::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  throw UnsupportedConstruct("When");
}

void When::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    pars = d.createCondition(f);
    pars->parse(f, ts, d);
  } else {
    ++f.c;
  }

  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    cond = d.createCondition(f);
    cond->parse(f, ts, d);
  } else {
    ++f.c;
  }

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
