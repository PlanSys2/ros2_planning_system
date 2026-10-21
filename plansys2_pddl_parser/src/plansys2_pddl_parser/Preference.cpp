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

#include <string>
#include <vector>

#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

void Preference::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( preference " << name << "\n";
  if (cond) {cond->PDDLPrint(s, indent + 1, ts, d);}
  s << "\n";
  tabindent(s, indent);
  s << ")";
}

plansys2_msgs::msg::Node::SharedPtr Preference::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  auto node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::PREFERENCE;
  node->node_id = tree.nodes.size();
  node->name = name;
  tree.nodes.push_back(*node);

  auto child = cond->getTree(tree, d, replace);
  tree.nodes[node->node_id].children.push_back(child->node_id);
  return node;
}

void Preference::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  // The name is optional
  f.next();
  if (f.getChar() != '(') {
    name = f.getToken();
    f.next();
  }
  f.assert_token("(");
  cond = d.createCondition(f);
  cond->parse(f, ts, d);
  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
