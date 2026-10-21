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
#include <memory>

#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

void Or::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( or\n";
  for (auto cond : conds) {
    cond->PDDLPrint(s, indent + 1, ts, d);
    s << "\n";
  }
  tabindent(s, indent);
  s << ")";
}

plansys2_msgs::msg::Node::SharedPtr Or::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::OR;
  node->node_id = tree.nodes.size();
  tree.nodes.push_back(*node);

  for (auto cond : conds) {
    plansys2_msgs::msg::Node::SharedPtr child = cond->getTree(tree, d, replace);
    tree.nodes[node->node_id].children.push_back(child->node_id);
  }

  return node;
}

void Or::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  for (f.next(); f.getChar() != ')'; f.next()) {
    f.assert_token("(");
    if (f.getChar() == ')') {
      throw std::runtime_error("Empty condition in or");
    }
    std::unique_ptr<Condition> condition(d.createCondition(f));
    condition->parse(f, ts, d);
    conds.push_back(condition.release());
  }
  ++f.c;
  if (conds.empty()) {
    throw std::runtime_error("An or needs at least one condition");
  }
}

}  // namespace pddl
}  // namespace parser
