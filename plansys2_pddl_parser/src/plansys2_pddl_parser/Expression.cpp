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
#include <string>
#include <vector>

#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace parser
{
namespace pddl
{

void FunctionExpression::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  ParamCond * c = d.funcs[d.funcs.index(fun->name)];

  s << "( " << fun->name;
  unsigned int psize = c->params.size();
  for (unsigned i = 0; i < psize; ++i) {
    if (ts.size() && fun->params[i] >= 0 && (unsigned)fun->params[i] < ts.size()) {
      s << " " << ts[fun->params[i]];
    } else if (fun->params[i] >= 0 && (unsigned)fun->params[i] >= ts.size()) {
      s << " ?" << fun->params[i];
    } else {
      // It is a constant parameter for the function
      s << " ";
      constants[i]->PDDLPrint(s, indent, ts, d);
    }
  }
  s << " )";
}

plansys2_msgs::msg::Node::SharedPtr FunctionExpression::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::FUNCTION;
  node->node_id = tree.nodes.size();
  node->name = fun->name;
  const int declared = d.funcs.index(fun->name);
  for (unsigned i = 0; i < fun->params.size(); ++i) {
    plansys2_msgs::msg::Param param;
    const int index = fun->params[i];
    ConstExpression * constant = nullptr;
    if (i < constants.size()) {
      constant = dynamic_cast<ConstExpression *>(constants[i]);
    }
    if (index >= 0 && static_cast<size_t>(index) < replace.size()) {
      param.name = replace[index];
    } else if (index < 0 && constant) {
      param.name = d.types[constant->tid]->constants[constant->constant];
    } else {
      param.name = "?" + std::to_string(index);
    }
    // The type the function declares for this argument
    if (declared >= 0 && i < d.funcs[declared]->params.size()) {
      param.type = d.types[d.funcs[declared]->params[i]]->name;
    }
    node->parameters.push_back(param);
  }
  tree.nodes.push_back(*node);
  return node;
}

double FunctionExpression::evaluate(Instance & ins, const StringVec & par)
{
  ParamCond * c = ins.d.funcs[ins.d.funcs.index(fun->name)];

  IntVec v(c->params.size());
  for (unsigned i = 0; i < v.size(); ++i) {
    std::pair<bool, unsigned> p = ins.d.types[c->params[i]]->parseObject(par[fun->params[i]]);
    if (p.first) {
      v[i] = p.second;
    } else {
      std::pair<bool, int> q = ins.d.types[c->params[i]]->parseConstant(par[fun->params[i]]);
      if (q.first) {
        v[i] = q.second;
      } else {
        return 1;
      }
    }
  }

  for (unsigned i = 0; i < ins.init.size(); ++i) {
    if (ins.init[i]->name == c->name && ins.init[i]->params == v) {
      return ((GroundFunc<double> *)ins.init[i])->value;
    }
  }
  return 1;
}

void ConstExpression::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  (void)indent;
  (void)ts;
  s << d.types[tid]->constants[constant];
}

plansys2_msgs::msg::Node::SharedPtr ConstExpression::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  (void)replace;
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::CONSTANT;
  node->node_id = tree.nodes.size();
  node->name = d.types[tid]->constants[constant];
  tree.nodes.push_back(*node);
  return node;
}

Expression * createExpression(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();

  if (f.getChar() == '(') {
    ++f.c;
    f.next();
    std::string s = f.getToken();
    if (s == "+" || s == "-" || s == "*" || s == "/") {
      // It is a composite expression
      std::unique_ptr<CompositeExpression> ce(new CompositeExpression(s));
      ce->parse(f, ts, d);
      return ce.release();
    } else if (s == "total-time") {
      f.next();
      f.assert_token(")");
      return new TotalTimeExpression();
    } else if (s == "is-violated") {
      f.next();
      const auto name = f.getToken();
      f.next();
      f.assert_token(")");
      return new ViolatedExpression(name);
    } else {
      // It is a function expression
      f.c -= s.size();
      Function * fun = d.funcs.get(f.getToken(d.funcs));
      // Owned by the expression once built; freed here if the text is wrong
      std::unique_ptr<FunctionExpression> expr(new FunctionExpression(new Lifted(fun)));
      for (unsigned i = 0; i < fun->params.size(); ++i) {
        f.next();
        std::string s = f.getToken();
        if (d.isConstant(s)) {
          expr->fun->params[i] = -1;
          IntPair ct = d.constantTypeIdConstId(s);
          expr->constants[i] = new ConstExpression(ct.first, ct.second);
        } else if (ts.index(s) >= 0) {
          expr->fun->params[i] = ts.index(s);
        } else {
          f.tokenExit(s);
        }
      }
      f.next();
      f.assert_token(")");
      return expr.release();
    }
  } else if (f.getChar() == '?') {
    // It is a parameter variable
    std::string s = f.getToken();

    int k = ts.index(s);
    if (k >= 0) {
      // It is a parameter variable
      return new ParamExpression(k);
    } else if (s == "?duration") {
      // It is a ?duration expression
      return new DurationExpression();
    } else {
      // Neither a known parameter variable nor ?duration
      f.tokenExit(s);
      return nullptr;
    }
  } else {
    std::string s = f.getToken();
    if (s.empty()) {
      // Nothing that is an expression (e.g. a ','): it would be read again, forever
      f.printLine();
      throw UnknownToken(std::string(1, f.getChar()));
    }

    if (d.funcs.index(s) >= 0 && d.funcs.get(s)->params.empty()) {
      // PDDL 2.1: a function without parameters may be written without parentheses
      return new FunctionExpression(new Lifted(d.funcs.get(s)));
    } else if (d.isConstant(s)) {
      // It is a constant!
      IntPair ct = d.constantTypeIdConstId(s);
      return new ConstExpression(ct.first, ct.second);
    } else if (s == "#t") {
      // The time of continuous effects (PDDL 2.1) and processes (PDDL+)
      return new TimeExpression();
    } else {
      // A number, whatever the locale, or an object of the problem, compared with an object
      // fluent
      double d = 0.0;
      if (!parseNumber(s, d) && isValidName(s)) {
        return new ObjectExpression(s);
      }
      if (!parseNumber(s, d)) {
        std::ostringstream os;
        os << "Expected a number, found \"" << s << "\"" << std::endl;
        f.printLine();
        std::cout << os.str();
        throw std::runtime_error(os.str());
      }
      return new ValueExpression(d);
    }
  }
}

}  // namespace pddl
}  // namespace parser
