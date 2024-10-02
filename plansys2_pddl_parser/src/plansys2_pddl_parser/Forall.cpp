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

void Forall::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( forall\n";

  TokenStruct<std::string> fstruct(ts);

  tabindent(s, indent + 1);
  printParams(0, s, fstruct, d);

  if (cond) {
    cond->PDDLPrint(s, indent + 1, fstruct, d);
  } else {
    tabindent(s, indent + 1);
    s << "()";
  }

  s << "\n";
  tabindent(s, indent);
  s << ")";
}

plansys2_msgs::msg::Node::SharedPtr Forall::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  throw UnsupportedConstruct("Forall");
}

void Forall::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token("(");

  TokenStruct<std::string> fs = f.parseTypedList(true, d.types);
  params = d.convertTypes(fs.types);

  TokenStruct<std::string> fstruct(ts);
  fstruct.append(fs);

  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    cond = d.createCondition(f);
    cond->parse(f, fstruct, d);
  } else {
    ++f.c;
  }

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
