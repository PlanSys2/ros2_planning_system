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
#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Condition.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace parser
{
namespace pddl
{

class Instance;

class Expression : public Condition
{
public:
  virtual ~Expression() {}
  virtual std::string info() const = 0;
  virtual double evaluate() = 0;
  virtual double evaluate(Instance & ins, const StringVec & par) = 0;
  virtual IntSet params() = 0;

  // inherit
  virtual void print(std::ostream & stream) const {stream << info();}

  virtual void parse(Stringreader &, TokenStruct<std::string> &, Domain &) {}
  virtual void addParams(int, unsigned) {}
};

Expression * createExpression(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

// (op a b ...): comparisons, - and / take two operands, + and * two or more, and - also one
class CompositeExpression : public Expression
{
public:
  std::string op;
  std::vector<Expression *> args;

  explicit CompositeExpression(const std::string & c)
  : op(c) {}

  CompositeExpression(const std::string & c, Expression * l, Expression * r)
  : op(c), args({l, r})
  {
  }

  CompositeExpression(const std::string & c, std::vector<Expression *> a)
  : op(c), args(std::move(a))
  {
  }

  ~CompositeExpression()
  {
    for (auto arg : args) {delete arg;}
  }

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
  {
    for (f.next(); f.getChar() != ')'; f.next()) {
      std::unique_ptr<Expression> arg(createExpression(f, ts, d));
      if (!arg) {
        throw std::runtime_error("Missing operand of " + op);
      }
      args.push_back(arg.release());
    }
    ++f.c;

    const bool multi = op == "+" || op == "*";
    const bool fits = multi ? args.size() >= 2 :
      (op == "-" ? args.size() == 1 || args.size() == 2 : args.size() == 2);
    if (!fits) {
      throw std::runtime_error(
              "Wrong number of operands (" + std::to_string(args.size()) + ") for " + op);
    }
  }

  std::string info() const
  {
    std::ostringstream os;
    os << "(" << op;
    for (auto arg : args) {os << " " << arg->info();}
    os << ")";
    return os.str();
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override
  {
    tabindent(s, indent);
    s << "( " << op;
    for (auto arg : args) {
      s << " ";
      arg->PDDLPrint(s, 0, ts, d);
    }
    s << " )";
  }

  // In the tree every operation is binary: (+ a b c) is (+ (+ a b) c) and (- a) is (- 0 a)
  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override
  {
    return binaryTree(tree, d, replace, args.size());
  }

  double compute(double x, double y)
  {
    double res = 0;

    if (op == "+") {
      res = x + y;
    } else if (op == "-") {
      res = x - y;
    } else if (op == "*") {
      res = x * y;
    } else if (op == "/") {
      res = (y == 0 ? 0 : x / y);
    }

    return res;
  }

  double evaluate()
  {
    if (args.size() == 1) {return compute(0, args[0]->evaluate());}
    double res = args[0]->evaluate();
    for (size_t i = 1; i < args.size(); ++i) {res = compute(res, args[i]->evaluate());}
    return res;
  }

  double evaluate(Instance & ins, const StringVec & par)
  {
    if (args.size() == 1) {return compute(0, args[0]->evaluate(ins, par));}
    double res = args[0]->evaluate(ins, par);
    for (size_t i = 1; i < args.size(); ++i) {res = compute(res, args[i]->evaluate(ins, par));}
    return res;
  }

  IntSet params()
  {
    IntSet pars;
    for (auto arg : args) {
      IntSet arg_pars = arg->params();
      pars.insert(arg_pars.begin(), arg_pars.end());
    }
    return pars;
  }

  Condition * copy(Domain & d)
  {
    std::vector<Expression *> copies;
    for (auto arg : args) {copies.push_back(dynamic_cast<Expression *>(arg->copy(d)));}
    return new CompositeExpression(op, copies);
  }

private:
  // The operation on the first n operands
  plansys2_msgs::msg::Node::SharedPtr binaryTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace,
    size_t n) const
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::EXPRESSION;
    node->expression_type = getExprType(op);
    node->node_id = tree.nodes.size();
    tree.nodes.push_back(*node);

    uint32_t left_id = 0;
    if (n == 1) {
      plansys2_msgs::msg::Node zero;
      zero.node_type = plansys2_msgs::msg::Node::NUMBER;
      zero.node_id = tree.nodes.size();
      zero.value = 0.0;
      tree.nodes.push_back(zero);
      left_id = zero.node_id;
    } else if (n == 2) {
      left_id = args[0]->getTree(tree, d, replace)->node_id;
    } else {
      left_id = binaryTree(tree, d, replace, n - 1)->node_id;
    }
    tree.nodes[node->node_id].children.push_back(left_id);

    auto right_child = args[n - 1]->getTree(tree, d, replace);
    tree.nodes[node->node_id].children.push_back(right_child->node_id);

    return node;
  }
};

class FunctionExpression : public Expression
{
public:
  ParamCond * fun;
  std::vector<Expression *> constants;

  explicit FunctionExpression(ParamCond * c)
  : fun(c)
  {
    constants.clear();
    constants.resize(c->params.size());
    std::fill(constants.begin(), constants.end(), nullptr);
  }
  FunctionExpression(ParamCond * c, std::vector<Expression *> e)
  : fun(c), constants(e) {}

  ~FunctionExpression()
  {
    delete fun;
    for (auto constant : constants) {delete constant;}
  }

  std::string info() const
  {
    std::ostringstream os;
    os << "(" << fun->name << fun->params << ")";
    return os.str();
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  double evaluate() {return 1;}

  double evaluate(Instance & ins, const StringVec & par);

  IntSet params() {return IntSet(fun->params.begin(), fun->params.end());}

  Condition * copy(Domain & d)
  {
    std::vector<Expression *> copies;
    for (auto constant : constants) {
      copies.push_back(constant ? dynamic_cast<Expression *>(constant->copy(d)) : nullptr);
    }
    return new FunctionExpression(dynamic_cast<ParamCond *>(fun->copy(d)), copies);
  }
};

class ValueExpression : public Expression
{
public:
  double value;

  explicit ValueExpression(double v)
  : value(v) {}

  std::string info() const
  {
    return numberToString(value, -1);
  }

  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> &,
    const Domain &) const override
  {
    s << numberToString(value, -1);
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::NUMBER;
    node->node_id = tree.nodes.size();
    node->value = value;
    tree.nodes.push_back(*node);
    return node;
  }

  double evaluate() {return value;}

  double evaluate(Instance &, const StringVec &) {return value;}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new ValueExpression(value);}
};

// #t, the time a continuous effect has acted: a PARAMETER node "#t", the step of its integration
class TimeExpression : public Expression
{
public:
  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> &,
    const Domain &) const override
  {
    s << "#t";
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::PARAMETER;
    node->node_id = tree.nodes.size();
    node->name = "#t";
    plansys2_msgs::msg::Param node_param;
    node_param.name = "#t";
    node->parameters.push_back(node_param);
    tree.nodes.push_back(*node);
    return node;
  }

  std::string info() const {return "#t";}

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new TimeExpression();}
};

class DurationExpression : public Expression
{
public:
  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> &,
    const Domain &) const override
  {
    s << "?duration";
  }

  // A parameter: the duration a plan gives the action, bound with bindDuration
  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::PARAMETER;
    node->node_id = tree.nodes.size();
    node->name = "?duration";
    plansys2_msgs::msg::Param node_param;
    node_param.name = "?duration";
    node->parameters.push_back(node_param);
    tree.nodes.push_back(*node);
    return node;
  }

  std::string info() const {return "?duration";}

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new DurationExpression();}
};

// (total-time): the makespan of the plan, only meaningful in a :metric
class TotalTimeExpression : public Expression
{
public:
  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> &,
    const Domain &) const override
  {
    s << "( total-time )";
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::FUNCTION;
    node->node_id = tree.nodes.size();
    node->name = "total-time";
    tree.nodes.push_back(*node);
    return node;
  }

  std::string info() const {return "total-time";}

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new TotalTimeExpression();}
};

// PDDL3: (is-violated name), how many times the preference called so is violated
class ViolatedExpression : public Expression
{
public:
  std::string name;

  explicit ViolatedExpression(const std::string & n)
  : name(n) {}

  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> &,
    const Domain &) const override
  {
    s << "( is-violated " << name << " )";
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    auto node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::FUNCTION;
    node->node_id = tree.nodes.size();
    node->name = "is-violated";
    plansys2_msgs::msg::Param param;
    param.name = name;
    node->parameters.push_back(param);
    tree.nodes.push_back(*node);
    return node;
  }

  std::string info() const {return "is-violated " + name;}

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new ViolatedExpression(name);}
};

class ParamExpression : public Expression
{
public:
  int param;

  explicit ParamExpression(int p)
  : param(p) {}

  std::string info() const
  {
    std::ostringstream os;
    os << param;
    return os.str();
  }

  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> & ts,
    const Domain &) const override
  {
    if (param >= 0 && static_cast<unsigned>(param) < ts.size()) {
      s << ts[param];
    } else {
      s << "?" << param;
    }
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & replace = {}) const override
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::PARAMETER;
    node->node_id = tree.nodes.size();
    node->name = "?" + std::to_string(param);
    plansys2_msgs::msg::Param node_param;
    if (static_cast<size_t>(replace.size()) > static_cast<size_t>(param)) {
      node_param.name = replace[param];
    } else {
      node_param.name = "?" + std::to_string(param);
    }
    node->parameters.push_back(node_param);
    tree.nodes.push_back(*node);
    return node;
  }

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new ParamExpression(param);}
};

class ConstExpression : public Expression
{
public:
  int constant;
  int tid;
  ConstExpression(int c, int t)
  : constant(c), tid(t) {}

  std::string info() const
  {
    std::ostringstream os;
    os << constant << " " << tid;
    return os.str();
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new ConstExpression(constant, tid);}
};

// An object of the problem in an expression, as in (= (pos ?r) kitchen): a CONSTANT node
class ObjectExpression : public Expression
{
public:
  std::string object;
  explicit ObjectExpression(const std::string & o)
  : object(o) {}

  std::string info() const {return object;}

  void PDDLPrint(
    std::ostream & s, unsigned, const TokenStruct<std::string> &, const Domain &) const override
  {
    s << object;
  }

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain &,
    const std::vector<std::string> & = {}) const override
  {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::CONSTANT;
    node->node_id = tree.nodes.size();
    node->name = object;
    tree.nodes.push_back(*node);
    return node;
  }

  double evaluate() {return -1;}

  double evaluate(Instance &, const StringVec &) {return evaluate();}

  IntSet params() {return IntSet();}

  Condition * copy(Domain &) {return new ObjectExpression(object);}
};

}  // namespace pddl
}  // namespace parser
