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

void Exists::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( exists (";

  for (unsigned i = 0; i < params.size(); ++i) {
    s << " " << (i < names.size() ? names[i] : "?" + std::to_string(params[i]));
    if (d.typed && i < types.size()) {s << " - " << d.types[types[i]]->name;}
  }
  s << " )\n";

  const TokenStruct<std::string> fstruct = printScope(ts, base, names);
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

plansys2_msgs::msg::Node::SharedPtr Exists::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::EXISTS;
  node->node_id = tree.nodes.size();
  // The variables keep their index as name, as the body refers to them
  for (unsigned i = 0; i < params.size(); ++i) {
    plansys2_msgs::msg::Param param;
    param.name = "?" + std::to_string(params[i]);
    if (i < types.size()) {
      param.type = d.types[types[i]]->name;
      d.types[types[i]]->getSubTypesNames(param.sub_types);
    }
    node->parameters.push_back(param);
  }

  tree.nodes.push_back(*node);
  plansys2_msgs::msg::Node::SharedPtr child;
  if (cond) {
    child = cond->getTree(tree, d, replace);
  } else {
    child = std::make_shared<plansys2_msgs::msg::Node>();
    child->node_type = plansys2_msgs::msg::Node::AND;
    child->node_id = tree.nodes.size();
    tree.nodes.push_back(*child);
  }
  tree.nodes[node->node_id].children.push_back(child->node_id);

  return node;
}

void Exists::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token("(");

  // The variables are only in the scope of the body
  TokenStruct<std::string> es = f.parseTypedList(true, d.types);
  assertUnique(es, "variable");
  base = ts.size();
  params = incvec(base, base + es.size());
  types = d.convertTypes(es.types);
  names = es.tokens;
  TokenStruct<std::string> scope = quantifierScope(ts, es);

  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    cond = d.createCondition(f);
    cond->parse(f, scope, d);
  } else {
    ++f.c;
  }

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
