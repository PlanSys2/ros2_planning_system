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
#include <cassert>

#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

Ground::Ground(const Ground * g, Domain & d)
: ParamCond(g), lifted(d.preds.get(g->name)), variables(g->variables) {}

void Ground::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( " << name;
  for (unsigned i = 0; i < params.size(); ++i) {
    if (i < variables.size() && variables[i]) {
      // A variable: its name in scope
      if (params[i] >= 0 && static_cast<unsigned>(params[i]) < ts.size()) {
        s << " " << ts[params[i]];
      } else {
        s << " ?" << params[i];
      }
    } else if (  // NOLINT
      params[i] >= 0 && d.types[lifted->params[i]]->object(params[i]).first != std::string(""))
    {
      s << " " << d.types[lifted->params[i]]->object(params[i]).first;
    } else if (params[i] < 0) {
      s << " " << d.types[lifted->params[i]]->object(params[i]).first;
    } else {
      s << " ?" + std::to_string(params[i]);
    }
  }
  s << " )";
}

plansys2_msgs::msg::Node::SharedPtr Ground::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  if (d.funcs.index(name) >= 0) {
    node->node_type = plansys2_msgs::msg::Node::FUNCTION;
  } else {
    node->node_type = plansys2_msgs::msg::Node::PREDICATE;
  }
  node->node_id = tree.nodes.size();
  node->name = name;
  for (unsigned i = 0; i < params.size(); ++i) {
    plansys2_msgs::msg::Param param;
    int param_index = params[i];
    const bool known = i < variables.size();
    bool is_variable = known ? variables[i] : param_index >= 0;

    if (is_variable) {
      // The index of the variable, not its position, says whether it is an action argument:
      // (at ?g ?z) in (secure ?z) has ?z second
      if (param_index >= 0 && static_cast<size_t>(param_index) < replace.size()) {
        // param has a variable value; replace by action-args
        param.name = replace[params[i]];
      } else if (known) {
        param.name = "?" + std::to_string(params[i]);
      } else if (!d.types[lifted->params[i]]->object(params[i]).first.empty()) {
        param.name = d.types[lifted->params[i]]->object(params[i]).first;
      } else {
        param.name = "?" + std::to_string(params[i]);
      }
    } else {
      // An object or a constant: its name
      param.name = d.types[lifted->params[i]]->object(params[i]).first;
    }

    // TODO(@rezenders): supertype is being parsed (i.e., object)
    param.type = d.types[lifted->params[i]]->name;
    node->parameters.push_back(param);
  }
  tree.nodes.push_back(*node);
  return node;
}

void Ground::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  params.resize(lifted->params.size());
  variables.assign(lifted->params.size(), false);
  for (unsigned i = 0; i < lifted->params.size(); ++i, f.next()) {
    std::string s = f.getToken();
    if (!s.empty() && s[0] == '?') {
      // A variable, also if an object had the same index
      int k = ts.index(s);
      if (k < 0) {
        f.tokenExit(s);
      }
      params[i] = k;
      variables[i] = true;
      continue;
    }
    std::pair<bool, int> p = d.types[lifted->params[i]]->parseConstant(s);
    if (p.first) {
      params[i] = p.second;
    } else {
      std::pair<bool, unsigned> q = d.types[lifted->params[i]]->parseObject(s);
      if (q.first) {
        params[i] = q.second;
      } else {
        int k = ts.index(s);
        if (k >= 0) {
          params[i] = k;
          variables[i] = true;  // A name in scope, printed as it is
        } else {
          f.tokenExit(s);
        }
      }
    }
  }
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
