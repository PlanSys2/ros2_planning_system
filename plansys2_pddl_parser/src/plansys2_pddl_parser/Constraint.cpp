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

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

bool Constraint::isOperator(const std::string & o)
{
  return arity(o).second > 0;
}

std::pair<unsigned, unsigned> Constraint::arity(const std::string & o)
{
  return constraintArity(o);
}

void Constraint::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( " << op;
  for (const auto time : times) {
    s << " " << numberToString(time);
  }
  s << "\n";
  for (auto * cond : conds) {
    cond->PDDLPrint(s, indent + 1, ts, d);
    s << "\n";
  }
  tabindent(s, indent);
  s << ")";
}

plansys2_msgs::msg::Node::SharedPtr Constraint::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  auto node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::CONSTRAINT;
  node->node_id = tree.nodes.size();
  node->name = op;
  tree.nodes.push_back(*node);

  for (const auto time : times) {
    plansys2_msgs::msg::Node number;
    number.node_type = plansys2_msgs::msg::Node::NUMBER;
    number.node_id = tree.nodes.size();
    number.value = time;
    tree.nodes.push_back(number);
    tree.nodes[node->node_id].children.push_back(number.node_id);
  }
  for (auto * cond : conds) {
    auto child = cond->getTree(tree, d, replace);
    tree.nodes[node->node_id].children.push_back(child->node_id);
  }
  return node;
}

void Constraint::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  const auto [n_times, n_conds] = arity(op);
  for (unsigned i = 0; i < n_times; i++) {
    f.next();
    const auto number = f.getToken();
    char * end = nullptr;
    const double time = std::strtod(number.c_str(), &end);
    if (number.empty() || end != number.c_str() + number.size() || time < 0) {
      throw std::runtime_error("(" + op + ") needs a time, not " + number);
    }
    times.push_back(time);
  }
  for (unsigned i = 0; i < n_conds; i++) {
    f.next();
    f.assert_token("(");
    conds.push_back(d.createCondition(f));
    conds.back()->parse(f, ts, d);
  }
  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
