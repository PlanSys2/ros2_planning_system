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

FunctionModifier::FunctionModifier(const std::string & name, int val)
: name(name), modifiedGround(0), modifierExpr(new ValueExpression(val))
{
}

FunctionModifier::FunctionModifier(const std::string & name, Function * f, const IntVec & p)
: name(name), modifiedGround(0), modifierExpr(new FunctionExpression(new Ground(f, p)))
{
}

FunctionModifier::FunctionModifier(const std::string & name, const FunctionModifier * i, Domain & d)
: name(name)
{
  if (i->modifiedGround) {
    modifiedGround = dynamic_cast<Ground *>(i->modifiedGround->copy(d));
  } else {
    modifiedGround = 0;
  }

  if (i->modifierExpr) {
    modifierExpr = dynamic_cast<Expression *>(i->modifierExpr->copy(d));
  } else {
    modifierExpr = 0;
  }
}

void FunctionModifier::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( " << name << " ";

  if (modifiedGround) {
    modifiedGround->PDDLPrint(s, 0, ts, d);
  } else {
    s << "( total-cost )";
  }

  s << " ";
  modifierExpr->PDDLPrint(s, 0, ts, d);

  s << " )";
}

plansys2_msgs::msg::Node::SharedPtr FunctionModifier::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::FUNCTION_MODIFIER;
  node->modifier_type = getFunModType(name);
  node->node_id = tree.nodes.size();
  tree.nodes.push_back(*node);

  if (modifiedGround) {
    plansys2_msgs::msg::Node::SharedPtr child = modifiedGround->getTree(tree, d, replace);
    tree.nodes[node->node_id].children.push_back(child->node_id);
  } else {
    // (total-cost), the function of :action-costs
    plansys2_msgs::msg::Node total_cost;
    total_cost.node_type = plansys2_msgs::msg::Node::FUNCTION;
    total_cost.name = "total-cost";
    total_cost.node_id = tree.nodes.size();
    tree.nodes.push_back(total_cost);
    tree.nodes[node->node_id].children.push_back(total_cost.node_id);
  }

  plansys2_msgs::msg::Node::SharedPtr child = modifierExpr->getTree(tree, d, replace);
  tree.nodes[node->node_id].children.push_back(child->node_id);

  return node;
}

void FunctionModifier::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();

  // The default value of the constructor is replaced by the one read
  delete modifiedGround;
  modifiedGround = 0;
  delete modifierExpr;
  modifierExpr = 0;

  if (f.getChar() != '(') {
    // PDDL 2.1: a function without parameters may be written without parentheses
    std::string increasedFunction = f.getToken();
    if (increasedFunction != "total-cost") {
      Function * function = d.funcs.get(increasedFunction);
      if (!function->params.empty()) {
        f.tokenExit(increasedFunction);
      }
      modifiedGround = new Ground(function);
      modifiedGround->variables.clear();
    }
  } else {
    f.assert_token("(");
    std::string increasedFunction = f.getToken();
    if (increasedFunction == "total-cost") {
      f.next();
      f.assert_token(")");
    } else {
      modifiedGround = new Ground(d.funcs.get(increasedFunction));
      modifiedGround->parse(f, ts, d);
    }
  }

  f.next();

  modifierExpr = createExpression(f, ts, d);
  if (!modifierExpr) {
    throw std::runtime_error("Missing value of " + name);
  }

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
