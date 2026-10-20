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

#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Expression.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
// #undef DOMAIN_DEBUG
// #define DOMAIN_DEBUG true
namespace parser
{
namespace pddl
{

class Instance
{
public:
  Domain & d;
  std::string name;
  GroundVec init;              // initial state

  // A timed initial literal or fluent: (at 10 (p)), (at 10 (not (p))), (at 10 (= (f) 3))
  struct TimedInit
  {
    double time;
    bool negated;
    TypeGround * fact;
  };
  std::vector<TimedInit> timedInit;
  Condition * goal = nullptr;  // Goal condition
  Condition * constraints = nullptr;  // PDDL3: what the plan of this problem keeps
  TokenStruct<std::string> ts;
  bool metric;                           // whether there is a :metric
  bool maximize = false;                 // maximize or minimize the metric
  Expression * metric_expr = nullptr;    // what the metric optimizes

  explicit Instance(Domain & dom)
  : d(dom), metric(false) {}

  Instance(Domain & dom, const std::string & s)
  : Instance(dom) {parse(s);}

  virtual ~Instance()
  {
    for (unsigned i = 0; i < init.size(); ++i) {delete init[i];}
    for (auto & timed : timedInit) {delete timed.fact;}
    if (nullptr != goal) {delete goal;}
    if (nullptr != constraints) {delete constraints;}
    if (nullptr != metric_expr) {delete metric_expr;}
  }

  void parse(const std::string & s)
  {
    // Deeper text would overflow the stack of the recursive parsers
    if (getMaxNesting(s) > kMaxNestingDepth) {
      throw std::runtime_error(
              "PDDL nested deeper than " + std::to_string(kMaxNestingDepth) + " levels");
    }
    Stringreader f(s);
    name = f.parseName("problem");

    if (DOMAIN_DEBUG) {std::cout << name << "\n";}

    for (; f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      f.assert_token(":");
      std::string t = f.getToken();

      if (DOMAIN_DEBUG) {std::cout << t << "\n";}

      if (t == "domain") {
        parseDomain(f);
      } else if (t == "objects") {
        parseObjects(f);
      } else if (t == "init") {
        parseInit(f);
      } else if (t == "goal") {
        parseGoal(f);
      } else if (t == "metric") {
        parseMetric(f);
      } else if (t == "constraints") {
        d.parseConstraints(f, constraints, ts);
      } else {
        f.tokenExit(t);
      }
    }
  }

  std::string getDomainName(const std::string & s)
  {
    std::string domain_name = "";

    Stringreader f(s);
    f.parseName("problem");

    for (; f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      f.assert_token(":");
      std::string t = f.getToken();
      if (t == "domain") {
        f.next();
        domain_name = f.getToken();
        break;
      }
    }
    return domain_name;
  }

  void parseDomain(Stringreader & f)
  {
    f.next();
    f.assert_token(d.name);
    f.assert_token(")");
  }

  void parseObjects(Stringreader & f)
  {
    // TokenStruct< std::string > ts = f.parseTypedList( true, d.types );

    // We need to populate the global ts with the objects read from the
    // object construct to ensure a correct reading in other places of
    // the code.
    ts = f.parseTypedList(true, d.types);

    for (unsigned i = 0; i < ts.size(); ++i) {
      Type * type = d.getType(ts.types[i]);
      std::pair<bool, unsigned> pair = type->parseObject(ts[i]);
      if (pair.first == false) {type->objects.insert(ts[i]);}
    }

    for (unsigned i = 0; DOMAIN_DEBUG && i < d.types.size(); ++i) {
      std::cout << " ";
      if (d.typed) {std::cout << " " << d.types[i] << ":";}
      for (unsigned j = 0; j < d.types[i]->objects.size(); ++j) {
        std::cout << " " << d.types[i]->objects[j];
      }
      std::cout << "\n";
    }
  }

  virtual void parseGround(Stringreader & f, GroundVec & v)
  {
    TypeGround * c = 0;
    if (f.getChar() == '=') {
      f.assert_token("=");
      f.assert_token("(");

      std::string s = f.getToken();
      int i = d.funcs.index(s);
      if (i < 0) {f.tokenExit(s);}

      if (d.funcs[i]->returnType < 0) {
        c = new GroundFunc<double>(d.funcs[i]);
      } else {
        c = new GroundFunc<int>(d.funcs[i]);
      }
    } else {
      c = new TypeGround(d.preds.get(f.getToken(d.preds)));
    }
    c->parse(f, d.types[0]->constants, d);
    v.push_back(c);
  }

  void parseInit(Stringreader & f)
  {
    for (f.next(); f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      if (!parseTimedInit(f)) {
        parseGround(f, init);
      }
    }
    ++f.c;

    for (unsigned i = 0; DOMAIN_DEBUG && i < init.size(); ++i) {std::cout << "  " << init[i];}
  }

  // (at <time> fact), its '(' read; false, and nothing read, if it is not one (a predicate at)
  bool parseTimedInit(Stringreader & f)
  {
    const auto mark = f.mark();
    if (f.getToken() != "at") {
      f.reset(mark);
      return false;
    }
    f.next();
    const auto number = f.getToken();
    char * end = nullptr;
    const double time = std::strtod(number.c_str(), &end);
    if (number.empty() || end != number.c_str() + number.size()) {
      f.reset(mark);
      return false;
    }
    if (time < 0) {
      throw std::runtime_error("A timed initial literal before the start: " + number);
    }
    f.next();
    f.assert_token("(");
    bool negated = false;
    const auto inner = f.c;
    if (f.getToken() == "not") {
      negated = true;
      f.next();
      f.assert_token("(");
    } else {
      f.c = inner;
    }
    GroundVec facts;
    parseGround(f, facts);
    if (negated) {
      f.next();
      f.assert_token(")");
    }
    f.next();
    f.assert_token(")");
    if (negated && (dynamic_cast<GroundFunc<double> *>(facts[0]) != nullptr ||
      dynamic_cast<GroundFunc<int> *>(facts[0]) != nullptr))
    {
      delete facts[0];
      throw std::runtime_error("A timed initial fluent cannot be negated");
    }
    timedInit.push_back({time, negated, reinterpret_cast<TypeGround *>(facts[0])});
    return true;
  }

  // add a timed initial literal or fluent
  void addTimedInit(double time, bool negated, TypeGround * fact)
  {
    timedInit.push_back({time, negated, fact});
  }

  virtual void parseGoal(Stringreader & f)
  {
    f.next();
    f.assert_token("(");
    if (f.getChar() != ')') {
      goal = d.createCondition(f);
      goal->parse(f, ts, d);
    }
    f.next();
    f.assert_token(")");
  }

  // (:metric minimize|maximize <numeric expression>), the expression may use (total-time)
  void parseMetric(Stringreader & f)
  {
    f.next();
    std::string optimization = f.getToken();
    if (optimization != "minimize" && optimization != "maximize") {
      f.tokenExit(optimization);
    }

    std::unique_ptr<Expression> expr(createExpression(f, ts, d));
    f.next();
    f.assert_token(")");

    if (nullptr != metric_expr) {delete metric_expr;}
    metric_expr = expr.release();
    maximize = optimization == "maximize";
    metric = true;
  }

  // add an object of a certain type
  void addObject(const std::string & name, const std::string & type)
  {
    // It is not enough to insert the name as an object of the given
    // type.  We need also to populate the TS, by insering the object
    // into the ts with the below two instructions to then inform
    // further calls to the parser, that there are the objects of the
    // given type.
    ts.insert(name);
    // The type of the new name only: ts.size() of them made k objects take k^2 / 2 strings
    ts.types.resize(ts.size(), type);
    d.getType(type)->objects.insert(name);
  }

  // add a predicate fluent to the initial state
  void addInit(const std::string & name, const StringVec & v = StringVec())
  {
    TypeGround * tg = new TypeGround(d.preds.get(name));
    tg->insert(d, v);
    init.push_back(tg);
  }

  // add a function value to the initial state
  void addInit(const std::string & name, int value, const StringVec & v = StringVec())
  {
    GroundFunc<int> * gf = new GroundFunc<int>(d.funcs.get(name), value);
    gf->insert(d, v);
    init.push_back(gf);
  }

  // add a function value to the initial state
  void addInit(const std::string & name, double value, const StringVec & v = StringVec())
  {
    GroundFunc<double> * gf = new GroundFunc<double>(d.funcs.get(name), value);
    gf->insert(d, v);
    init.push_back(gf);
  }

  // add the object of an object fluent to the initial state
  void addInitObject(const std::string & name, const std::string & object, const StringVec & v)
  {
    Function * function = d.funcs.get(name);
    std::pair<bool, unsigned> p = d.types[function->returnType]->parseObject(object);
    int index = static_cast<int>(p.second);
    if (!p.first) {
      std::pair<bool, int> q = d.types[function->returnType]->parseConstant(object);
      if (!q.first) {
        throw std::runtime_error("Unknown object " + object + " of " + name);
      }
      index = q.second;
    }
    GroundFunc<int> * gf = new GroundFunc<int>(function, index);
    gf->insert(d, v);
    init.push_back(gf);
  }

  // add a fluent (predicate or function) to the initial state
  void addInit(TypeGround * g, const StringVec & v = StringVec())
  {
    TypeGround * tg = 0;
    GroundFunc<int> * f1 = dynamic_cast<GroundFunc<int> *>(g);
    GroundFunc<double> * f2 = dynamic_cast<GroundFunc<double> *>(g);
    if (f1) {
      tg = new GroundFunc<int>(d.funcs.get(g->name), f1->value);
    } else if (f2) {
      tg = new GroundFunc<double>(d.funcs.get(g->name), f2->value);
    } else {
      tg = new TypeGround(d.preds.get(g->name));
    }
    tg->insert(d, v);
    init.push_back(tg);
  }

  // Add the goal represented by s to the Instance
  void addGoal(const std::string & s)
  {
    // In order to invoke the parseGoal, the input string shall also
    // contain an ")" for simulating the right parenthesis of "(goal
    // ....)", and an additional left ")" parenthesis for the part
    // outside. We need to handle also the empty goal case
    //
    // Alternatively, one can define another parseGoal function
    // that does not require these two additional characters.
    std::string ss = "";
    if (0 == s.length()) {
      ss = "(and ) ))";
    } else {
      ss = s + "))";
    }
    Stringreader f(ss);
    parseGoal(f);
  }

  // Set the constraints from their condition, s; an empty one removes them
  void addConstraints(const std::string & s)
  {
    if (s.empty()) {
      if (constraints) {delete constraints;}
      constraints = nullptr;
      return;
    }
    Stringreader f(s + ")");
    d.parseConstraints(f, constraints, ts);
  }

  // Set the metric: optimization is "minimize" or "maximize", s a numeric expression
  void addMetric(const std::string & optimization, const std::string & s)
  {
    std::string ss = optimization + " " + s + ")";
    Stringreader f(ss);
    parseMetric(f);
  }

  friend std::ostream & operator<<(std::ostream & os, const Instance & o) {return o.print(os);}
  virtual std::ostream & print(std::ostream & stream) const
  {
    stream << "( define ( problem " << name << " )\n";
    stream << "( :domain " << d.name << " )\n";

    stream << "( :objects\n";
    for (unsigned i = 0; i < d.types.size(); ++i) {
      if (d.types[i]->objects.size()) {
        stream << "\t";
        for (unsigned j = 0; j < d.types[i]->objects.size(); ++j) {
          stream << d.types[i]->objects[j] << " ";
        }
        if (d.typed) {stream << "- " << d.types[i]->name;}
        stream << "\n";
      }
    }
    stream << ")\n";

    stream << "( :init\n";
    for (unsigned i = 0; i < init.size(); ++i) {
      (reinterpret_cast<TypeGround *>(init[i]))->PDDLPrint(
        stream, 1, TokenStruct<std::string>(), d);
      stream << "\n";
    }
    for (const auto & timed : timedInit) {
      std::ostringstream fact;
      timed.fact->PDDLPrint(fact, 0, TokenStruct<std::string>(), d);
      // Six decimals: the default precision of a stream drops them in long problems
      stream << "\t( at " << std::to_string(timed.time) << " " <<
        (timed.negated ? "( not " + fact.str() + " )" : fact.str()) << " )\n";
    }
    stream << ")\n";

    stream << "( :goal\n";
    if (nullptr != goal) {goal->PDDLPrint(stream, 1, ts /* TokenStruct< std::string >() */, d);}
    stream << ")\n";

    if (nullptr != constraints) {
      stream << "( :constraints\n";
      constraints->PDDLPrint(stream, 1, ts, d);
      stream << "\n)\n";
    }

    if (metric && nullptr != metric_expr) {
      stream << "( :metric " << (maximize ? "maximize" : "minimize") << " ";
      metric_expr->PDDLPrint(stream, 0, ts, d);
      stream << " )\n";
    }

    stream << ")\n";
    return stream;
  }
};

}  // namespace pddl
}  // namespace parser
