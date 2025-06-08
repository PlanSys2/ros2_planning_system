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

void Imply::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( imply\n";

  TokenStruct<std::string> fstruct(ts);

  tabindent(s, indent + 1);
  printParams(0, s, fstruct, d);

  if (left) {left->PDDLPrint(s, indent + 1, fstruct, d);}
  if (right) {right->PDDLPrint(s, indent + 1, fstruct, d);} else {
    tabindent(s, indent + 1);
    s << "()";
  }

  s << "\n";
  tabindent(s, indent);
  s << ")";
}

plansys2_msgs::msg::Node::SharedPtr Imply::getTree(
  plansys2_msgs::msg::Tree & tree,
  const Domain & d, const std::vector<std::string> & replace,
  const std::map<std::string, std::vector<std::string>> & instances_map) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::IMPLY;
  node->node_id = tree.nodes.size();
  node->name = name;

  for (unsigned i = 0; i < params.size(); ++i) {
    plansys2_msgs::msg::Param param;
    if (i < replace.size()) {
      if (params[i] >= 0) {  // param has a variable value; replace by action-args
        param.name = replace[i];
      }
    } else {
      param.name = "?" + std::to_string(params[i]);
    }
    param.type = d.types[params[i]]->name;
    node->parameters.push_back(param);
  }

  tree.nodes.push_back(*node);

  if (left) {
    plansys2_msgs::msg::Node::SharedPtr child_1 = left->getTree(tree, d, replace);
    // Could conditions need instances?
    tree.nodes[node->node_id].children.push_back(child_1->node_id);
  }

  if(right) {
    plansys2_msgs::msg::Node::SharedPtr child_2 = right->getTree(tree, d, replace);
    // Could conditions need instances?
    tree.nodes[node->node_id].children.push_back(child_2->node_id);
  }
  return node;
}

void Imply::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token("(");
  params = d.convertTypes(ts.types);
  if (f.getChar() != ')') {
    left = d.createCondition(f);
    left->parse(f, ts, d);
  } else {++f.c;}

  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    right = d.createCondition(f);
    right->parse(f, ts, d);
  } else {++f.c;}

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
