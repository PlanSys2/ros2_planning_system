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

#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include "plansys2_pddl_parser/And.hpp"
#include "plansys2_pddl_parser/Constraint.hpp"
#include "plansys2_pddl_parser/Derived.hpp"
#include "plansys2_pddl_parser/EitherType.hpp"
#include "plansys2_pddl_parser/Exists.hpp"
#include "plansys2_pddl_parser/Forall.hpp"
#include "plansys2_pddl_parser/Function.hpp"
#include "plansys2_pddl_parser/FunctionModifier.hpp"
#include "plansys2_pddl_parser/GroundFunc.hpp"
#include "plansys2_pddl_parser/Imply.hpp"
#include "plansys2_pddl_parser/Not.hpp"
#include "plansys2_pddl_parser/Oneof.hpp"
#include "plansys2_pddl_parser/Or.hpp"
#include "plansys2_pddl_parser/Preference.hpp"
#include "plansys2_pddl_parser/Task.hpp"
#include "plansys2_pddl_parser/TemporalAction.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_pddl_parser/When.hpp"

#define DOMAIN_DEBUG false

namespace parser
{
namespace pddl
{

class Domain
{
public:
  std::string name;  // name of domain

  bool equality;                  // whether domain supports equality
  bool strips, adl, condeffects;  // whether domain is strips, adl and/or has conditional effects
  bool typed, cons, costs;        // whether domain is typed, has constants, has costs
  bool temp,  // whether domain is temporal
    nondet,  // whether domain is is non-deterministic
    neg,  // whether domain has negative precons
    disj;  // whether domain has disjunctive preconditions
  bool universal;        // whether domain has universal precons
  bool fluents;          // whether domains contains fluents
  bool derivedpred;      // whether domain contains derived predicates
  bool existentialcond;  // whether domain contains existential predicates
  StringVec requirements;  // as written, to print them back

  TokenStruct<Type *> types;       // types
  TokenStruct<Lifted *> preds;     // predicates
  TokenStruct<Function *> funcs;   // functions
  TokenStruct<Action *> actions;   // actions
  TokenStruct<Derived *> derived;  // derived predicates
  TokenStruct<Action *> processes;  // PDDL+: what acts while its precondition holds
  TokenStruct<Action *> events;     // PDDL+: what happens when its precondition becomes true
  TokenStruct<Task *> tasks;       // tasks
  Condition * constraints = nullptr;  // PDDL3: what every plan of every problem keeps

  Domain()
  : equality(false),
    strips(false),
    adl(false),
    condeffects(false),
    typed(false),
    cons(false),
    costs(false),
    temp(false),
    nondet(false),
    neg(false),
    disj(false),
    universal(false),
    fluents(false),
    derivedpred(false),
    existentialcond(false)
  {
    types.insert(
      new Type("object"));  // Type 0 is always "object", whether the domain is typed or not
  }

  explicit Domain(const std::string & s)
  : Domain() {parse(s);}

  virtual ~Domain()
  {
    for (unsigned i = 0; i < types.size(); ++i) {delete types[i];}
    for (unsigned i = 0; i < preds.size(); ++i) {delete preds[i];}
    for (unsigned i = 0; i < funcs.size(); ++i) {delete funcs[i];}
    for (unsigned i = 0; i < actions.size(); ++i) {delete actions[i];}
    for (unsigned i = 0; i < derived.size(); ++i) {delete derived[i];}
    for (unsigned i = 0; i < processes.size(); ++i) {delete processes[i];}
    for (unsigned i = 0; i < events.size(); ++i) {delete events[i];}
    for (unsigned i = 0; i < tasks.size(); ++i) {delete tasks[i];}
    if (constraints) {delete constraints;}
  }

  virtual void parse(const std::string & s)
  {
    // Deeper text would overflow the stack of the recursive parsers
    if (getMaxNesting(s) > kMaxNestingDepth) {
      throw std::runtime_error(
              "PDDL nested deeper than " + std::to_string(kMaxNestingDepth) + " levels");
    }
    Stringreader f(s);
    name = f.parseName("domain");

    if (DOMAIN_DEBUG) {std::cout << name << "\n";}

    for (; f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      f.assert_token(":");
      std::string t = f.getToken();

      if (DOMAIN_DEBUG) {std::cout << t << "\n";}

      if (!parseBlock(t, f)) {
        f.tokenExit(t);
      }
    }
  }

  //! Returns a boolean indicating whether the block was correctly parsed
  virtual bool parseBlock(const std::string & t, Stringreader & f)
  {
    if (t == "requirements") {
      parseRequirements(f);
    } else if (t == "types") {
      parseTypes(f);
    } else if (t == "constants") {
      parseConstants(f);
    } else if (t == "predicates") {
      parsePredicates(f);
    } else if (t == "functions") {
      parseFunctions(f);
    } else if (t == "action") {
      parseAction(f);
    } else if (t == "durative-action") {
      parseDurativeAction(f);
    } else if (t == "process" || t == "event") {
      parseHappening(f, t);
    } else if (t == "derived") {
      parseDerived(f);
    } else if (t == "constraints") {
      parseConstraints(f, constraints);
    } else {
      return false;  // Unknown block type
    }
    // else if ( t == "axiom" ) parseAxiom( f );
    return true;
  }

  // (:constraints C), its "(:constraints" read; an empty one, (:constraints), is none. The
  // names of a problem go in ts, as in its goal
  void parseConstraints(Stringreader & f, Condition *& into)
  {
    TokenStruct<std::string> ts;
    parseConstraints(f, into, ts);
  }

  void parseConstraints(Stringreader & f, Condition *& into, TokenStruct<std::string> & ts)
  {
    f.next();
    if (f.getChar() != ')') {
      f.assert_token("(");
      std::unique_ptr<Condition> cond(createCondition(f));
      cond->parse(f, ts, *this);
      if (into) {delete into;}
      into = cond.release();
    }
    f.next();
    f.assert_token(")");
  }

  void parseRequirements(Stringreader & f)
  {
    for (f.next(); f.getChar() != ')'; f.next()) {
      f.assert_token(":");
      std::string s = f.getToken();

      if (DOMAIN_DEBUG) {std::cout << "  " << s << "\n";}

      if (!parseRequirement(s)) {
        f.tokenExit(s);
      }
    }

    ++f.c;
  }

  // Standard requirements without a flag: what they need is checked where it is used
  static bool isStandardRequirement(const std::string & s)
  {
    return s == "duration-inequalities" || s == "continuous-effects" ||
           s == "timed-initial-literals" || s == "preferences" || s == "constraints" ||
           s == "time";
  }

  //! Returns a boolean indicating whether the requirement was correctly parsed
  virtual bool parseRequirement(const std::string & s)
  {
    if (s == "strips") {
      strips = true;
    } else if (s == "adl") {
      adl = true;
    } else if (s == "negative-preconditions") {
      neg = true;
    } else if (s == "conditional-effects") {
      condeffects = true;
    } else if (s == "typing") {
      typed = true;
    } else if (s == "action-costs" || s == "action-cost") {
      costs = true;
    } else if (s == "equality") {
      equality = true;
    } else if (s == "durative-actions") {
      temp = true;
    } else if (s == "non-deterministic") {
      nondet = true;
    } else if (s == "universal-preconditions") {
      universal = true;
    } else if (s == "fluents" || s == "numeric-fluents" || s == "object-fluents") {
      fluents = true;
    } else if (s == "disjunctive-preconditions") {
      disj = true;
    } else if (s == "derived-predicates") {
      derivedpred = true;
    } else if (s == "existential-preconditions") {
      existentialcond = true;
    } else if (s == "quantified-preconditions") {
      existentialcond = universal = true;
    } else if (!isStandardRequirement(s)) {
      return false;  // Unknown requirement
    }
    requirements.push_back(s);
    return true;
  }

  // get the type corresponding to a string
  Type * getType(std::string s)
  {
    int i = types.index(s);
    if (i < 0) {
      if (s[0] == '(') {
        i = types.insert(new EitherType(s));
        for (unsigned k = 9; s[k] != ')'; ) {
          unsigned e = s.find(' ', k);
          types[i]->subtypes.push_back(getType(s.substr(k, e - k)));
          k = e + 1;
        }
      } else {
        i = types.insert(new Type(s));
      }
    }
    return types[i];
  }

  // convert a vector of type names to integers
  IntVec convertTypes(const StringVec & v)
  {
    IntVec out;
    for (unsigned i = 0; i < v.size(); ++i) {out.push_back(types.index(getType(v[i])->name));}
    return out;
  }

  void parseTypes(Stringreader & f)
  {
    // Many domains declare types without :typing: the types say it
    typed = true;

    // if this makes it in, probably need to define new subclass of Type
    // if ( costs ) insert( new Type( "NUMBER" ), tmap, types );

    // Parse the typed list
    // A type may be listed more than once (IPC storage does); a cycle is an error below
    TokenStruct<std::string> ts = f.parseTypedList(false);

    // bit of a hack to avoid object being the supertype
    if (ts.index("object") >= 0) {
      types[0]->name = "supertype";
      types.tokenMap.clear();
      types.tokenMap["supertype"] = 0;
    }

    // Relate subtypes and supertypes
    for (unsigned i = 0; i < ts.size(); ++i) {
      if (std::find(types.types.begin(), types.types.end(), ts.types[i]) == types.types.end()) {
        if (ts.types[i].size()) {
          Type * supertype = getType(ts.types[i]);
          Type * subtype = getType(ts[i]);
          // A cycle in the subtypes is not always one of supertypes, as these are replaced
          if (subtype->reaches(supertype)) {
            throw std::runtime_error("Cycle of types with \"" + subtype->name + "\"");
          }
          supertype->insertSubtype(subtype);
        } else {
          getType(ts[i]);
        }
      }
    }

    // A type can not be its own supertype, directly or not
    for (unsigned i = 1; i < types.size(); ++i) {
      std::set<const Type *> chain;
      for (const Type * t = types[i]; t != 0; t = t->supertype) {
        if (!chain.insert(t).second) {
          throw std::runtime_error("Cycle of types with \"" + types[i]->name + "\"");
        }
      }
    }

    // By default, the supertype of a type is "object"; not of one that already reaches it, as
    // (either object) does through its alternatives: that would be a cycle
    for (unsigned i = 1; i < types.size(); ++i) {
      if (types[i]->supertype == 0 && !types[i]->reaches(types[0])) {
        types[0]->insertSubtype(types[i]);
      }
    }

    for (unsigned i = 0; DOMAIN_DEBUG && i < types.size(); ++i) {std::cout << "  " << types[i];}
  }

  void parseConstants(Stringreader & f)
  {
    if (typed && !types.size()) {
      throw std::runtime_error("Types needed before defining constants");
    }

    cons = true;

    TokenStruct<std::string> ts = f.parseTypedList(true, types);
    assertUnique(ts, "constant");

    for (unsigned i = 0; i < ts.size(); ++i) {
      if (isConstant(ts[i])) {
        throw std::runtime_error("Duplicate constant \"" + ts[i] + "\"");
      }
      Type * t = getType(ts.types[i]);
      t->constants.insert(ts[i]);
    }
    for (unsigned i = 0; DOMAIN_DEBUG && i < types.size(); ++i) {
      std::cout << " ";
      if (typed) {std::cout << " " << types[i] << ":";}
      for (unsigned j = 0; j < types[i]->constants.size(); ++j) {
        std::cout << " " << types[i]->constants[j];
      }
      std::cout << "\n";
    }
  }

  void parsePredicates(Stringreader & f)
  {
    if (typed && !types.size()) {
      throw std::runtime_error("Types needed before defining predicates");
    }

    for (f.next(); f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      if (f.getChar() == ':') {
        // Needed to support MA-PDDL
        f.assert_token(":private");
        f.parseTypedList(true, types, "(");

        // CURRENT HACK: TOTALLY IGNORE PRIVATE !!!
        --f.c;
        parsePredicates(f);
      } else {
        std::unique_ptr<Lifted> c(new Lifted(f.getName()));
        if (preds.index(c->name) >= 0) {
          throw std::runtime_error("Duplicate predicate \"" + c->name + "\"");
        }
        c->parse(f, types[0]->constants, *this);

        if (DOMAIN_DEBUG) {std::cout << "  " << c.get();}
        preds.insert(c.release());
      }
    }
    ++f.c;
  }

  void parseFunctions(Stringreader & f)
  {
    if (typed && !types.size()) {
      throw std::runtime_error("Types needed before defining functions");
    }

    for (f.next(); f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      std::unique_ptr<Function> c(new Function(f.getName()));
      if (funcs.index(c->name) >= 0) {
        throw std::runtime_error("Duplicate function \"" + c->name + "\"");
      }
      c->parse(f, types[0]->constants, *this);

      if (DOMAIN_DEBUG) {std::cout << "  " << c.get();}
      funcs.insert(c.release());
    }
    ++f.c;
  }

  virtual void parseAction(Stringreader & f)
  {
    f.next();
    std::unique_ptr<Action> a(new Action(f.getName()));
    if (actions.index(a->name) >= 0) {
      throw std::runtime_error("Duplicate action \"" + a->name + "\"");
    }
    a->parse(f, types[0]->constants, *this);

    if (DOMAIN_DEBUG) {std::cout << a.get() << "\n";}
    actions.insert(a.release());
  }

  // A process or an event: parameters, precondition and effect, as an action
  void parseHappening(Stringreader & f, const std::string & keyword)
  {
    f.next();
    std::unique_ptr<Action> a(new Action(f.getName()));
    a->keyword = keyword;
    auto & list = keyword == "process" ? processes : events;
    if (list.index(a->name) >= 0) {
      throw std::runtime_error("Duplicate " + keyword + " \"" + a->name + "\"");
    }
    a->parse(f, types[0]->constants, *this);
    list.insert(a.release());
  }

  void parseDerived(Stringreader & f)
  {
    if (!preds.size()) {
      throw std::runtime_error("Predicates needed before defining derived predicates");
    }

    f.next();
    std::unique_ptr<Derived> d(new Derived);
    d->parse(f, types[0]->constants, *this);

    if (DOMAIN_DEBUG) {std::cout << d.get() << "\n";}
    derived.insert(d.release());
  }

  void parseDurativeAction(Stringreader & f)
  {
    f.next();
    std::unique_ptr<Action> a(new TemporalAction(f.getName()));
    if (actions.index(a->name) >= 0) {
      throw std::runtime_error("Duplicate action \"" + a->name + "\"");
    }
    a->parse(f, types[0]->constants, *this);

    if (DOMAIN_DEBUG) {std::cout << a.get() << "\n";}
    actions.insert(a.release());
  }

  // Return a copy of the type structure, with newly allocated types
  // This will also copy all constants and objects!
  TokenStruct<Type *> copyTypes()
  {
    TokenStruct<Type *> out;
    for (unsigned i = 0; i < types.size(); ++i) {out.insert(types[i]->copy());}

    for (unsigned i = 1; i < types.size(); ++i) {
      if (types[i]->supertype) {
        out[out.index(types[i]->supertype->name)]->insertSubtype(out[i]);
      } else {
        out[i]->copySubtypes(types[i], out);
      }
    }

    return out;
  }

  // Set the types to "otherTypes"
  void setTypes(const TokenStruct<Type *> & otherTypes)
  {
    for (unsigned i = 0; i < types.size(); ++i) {delete types[i];}
    types = otherTypes;
  }

  // Create a type with a given supertype (default is "object")
  void createType(const std::string & name, const std::string & parent = "object")
  {
    Type * type = new Type(name);
    types.insert(type);
    types.get(parent)->insertSubtype(type);
  }

  // Create a constant of a given type
  void createConstant(const std::string & name, const std::string & type)
  {
    types.get(type)->constants.insert(name);
  }

  // Create a predicate with the given name and parameter types
  Lifted * createPredicate(const std::string & name, const StringVec & params = StringVec())
  {
    Lifted * pred = new Lifted(name);
    for (unsigned i = 0; i < params.size(); ++i) {pred->params.push_back(types.index(params[i]));}
    preds.insert(pred);
    return pred;
  }

  // Create a function with the given name and parameter types
  Lifted * createFunction(
    const std::string & name, int type, const StringVec & params = StringVec())
  {
    Function * func = new Function(name, type);
    for (unsigned i = 0; i < params.size(); ++i) {func->params.push_back(types.index(params[i]));}
    funcs.insert(func);
    return func;
  }

  // Create an action with the given name and parameter types
  Action * createAction(const std::string & name, const StringVec & params = StringVec())
  {
    Action * action = new Action(name);
    for (unsigned i = 0; i < params.size(); ++i) {action->params.push_back(types.index(params[i]));}
    action->pre = new And;
    action->eff = new And;
    actions.insert(action);
    return action;
  }

  // Set the precondition of an action to "cond", converting to "And"
  void setPre(const std::string & act, Condition * cond)
  {
    Action * action = actions.get(act);

    And * old = dynamic_cast<And *>(cond);
    if (old == 0) {
      action->pre = new And;
      if (cond) {dynamic_cast<And *>(action->pre)->add(cond->copy(*this));}
    } else {
      action->pre = old->copy(*this);
    }
  }

  // Add a precondition to the action with name "act"
  void addPre(
    bool neg, const std::string & act, const std::string & pred, const IntVec & params = IntVec())
  {
    Action * action = actions.get(act);
    if (action->pre == 0) {action->pre = new And;}
    And * a = dynamic_cast<And *>(action->pre);
    if (neg) {
      a->add(new Not(ground(pred, params)));
    } else {
      a->add(ground(pred, params));
    }
  }

  // Add an "or" precondition to the action with name "act"
  void addOrPre(
    const std::string & act, const std::string & pred1, const std::string & pred2,
    const IntVec & params1 = IntVec(), const IntVec & params2 = IntVec())
  {
    Or * o = new Or;
    o->add(ground(pred1, params1));
    o->add(ground(pred2, params2));
    Action * action = actions.get(act);
    And * a = dynamic_cast<And *>(action->pre);
    a->add(o);
  }

  // Set the precondition of an action to "cond", converting to "And"
  void setEff(const std::string & act, Condition * cond)
  {
    Action * action = actions.get(act);

    And * old = dynamic_cast<And *>(cond);
    if (old == 0) {
      action->eff = new And;
      if (cond) {dynamic_cast<And *>(action->eff)->add(cond->copy(*this));}
    } else {
      action->eff = old->copy(*this);
    }
  }

  // Add an effect to the action with name "act"
  void addEff(
    bool neg, const std::string & act, const std::string & pred, const IntVec & params = IntVec())
  {
    Action * action = actions.get(act);
    if (action->eff == 0) {action->eff = new And;}
    And * a = dynamic_cast<And *>(action->eff);
    if (neg) {
      a->add(new Not(ground(pred, params)));
    } else {
      a->add(ground(pred, params));
    }
  }

  // Add a cost to the action with name "act", in the form of an integer
  void addCost(const std::string & act, int cost)
  {
    Action * action = actions.get(act);
    if (action->eff == 0) {action->eff = new And;}
    And * a = dynamic_cast<And *>(action->eff);
    a->add(new Increase(cost));
  }

  // Add a cost to the action with name "act", in the form of a function
  void addCost(const std::string & act, const std::string & func, const IntVec & params = IntVec())
  {
    Action * action = actions.get(act);
    if (action->eff == 0) {action->eff = new And;}
    And * a = dynamic_cast<And *>(action->eff);
    a->add(new Increase(funcs.get(func), params));
  }

  void addFunctionModifier(const std::string & act, FunctionModifier * fm)
  {
    Action * action = actions.get(act);
    if (action->eff == 0) {action->eff = new And;}
    And * a = dynamic_cast<And *>(action->eff);
    a->add(fm);
  }

  // Create a ground condition with the given name
  Ground * ground(const std::string & name, const IntVec & params = IntVec())
  {
    if (preds.index(name) < 0) {
      throw std::runtime_error(
              "Creating a ground condition " + name +
              " failed since the predicate does not exist");
    }
    return new Ground(preds[preds.index(name)], params);
  }

  // Return the list of type names corresponding to a parameter list
  StringVec typeList(ParamCond * c)
  {
    StringVec out;
    for (unsigned i = 0; i < c->params.size(); ++i) {out.push_back(types[c->params[i]]->name);}
    return out;
  }

  // Return the list of object names corresponding to a ground fluent
  StringVec objectList(Ground * g)
  {
    StringVec out;
    for (unsigned i = 0; i < g->params.size(); ++i) {
      out.push_back(types[g->lifted->params[i]]->object(g->params[i]).first);
    }
    return out;
  }

  // Add parameters to an action
  void addParams(const std::string & name, const StringVec & v)
  {
    actions.get(name)->addParams(convertTypes(v));
  }

  // Assert that one type is a subtype of another
  bool assertSubtype(int t1, int t2)
  {
    for (Type * type = types[t1]; type != 0; type = type->supertype) {
      if (type->name == types[t2]->name) {return 1;}}
    return 0;
  }

  // Check whether the given string represents a valid constant in the model
  bool isConstant(const std::string & name)
  {
    bool res = false;
    for (unsigned t = 0; t < types.size() && !res; t++) {
      for (unsigned c = 0; c < types[t]->constants.size() && !res; c++) {
        if (types[t]->constants[c] == name) {res = true;}
      }
    }
    return res;
  }

  IntPair constantTypeIdConstId(const std::string & name)
  {
    int t = 0, c = 0;
    bool found = false;
    for (t = 0; t < static_cast<int>(types.size()) && !found; t++) {
      for (c = 0; c < static_cast<int>(types[t]->constants.size()) && !found; c++) {
        if (name == types[t]->constants[c]) {found = true;}
      }
    }
    if (!found) {return IntPair(-1, -1);}
    return IntPair(c - 1, t - 1);
  }

  // return the index of a constant for a given type
  int constantIndex(const std::string & name, const std::string & type)
  {
    return types.get(type)->parseConstant(name).second;
  }

  //! Prints a PDDL representation of the object to the given stream.
  friend std::ostream & operator<<(std::ostream & os, const Domain & o) {return o.print(os);}
  virtual std::ostream & print(std::ostream & os) const
  {
    os << "( define ( domain " << name << " )\n";
    print_requirements(os);

    if (typed) {
      os << "( :types\n";
      for (unsigned i = 1; i < types.size(); ++i) {types[i]->PDDLPrint(os);}
      os << ")\n";
    }

    if (cons) {
      os << "( :constants\n";
      for (unsigned i = 0; i < types.size(); ++i) {
        for (unsigned j = 0; j < types[i]->constants.size(); j++) {
          if (!types[i]->definedInSubtype(types[i]->constants[j])) {
            os << "\t" << types[i]->constants[j] << " ";
            if (typed) {os << "- " << types[i]->name;}
            os << "\n";
          }
        }
      }
      os << ")\n";
    }

    os << "( :predicates\n";
    for (unsigned i = 0; i < preds.size(); ++i) {
      preds[i]->PDDLPrint(os, 1, TokenStruct<std::string>(), *this);
      os << "\n";
    }
    os << ")\n";

    if (funcs.size()) {
      os << "( :functions\n";
      for (unsigned i = 0; i < funcs.size(); ++i) {
        funcs[i]->PDDLPrint(os, 1, TokenStruct<std::string>(), *this);
        os << "\n";
      }
      os << ")\n";
    }

    for (unsigned i = 0; i < actions.size(); ++i) {
      actions[i]->PDDLPrint(os, 0, TokenStruct<std::string>(), *this);
    }

    for (unsigned i = 0; i < derived.size(); ++i) {
      derived[i]->PDDLPrint(os, 0, TokenStruct<std::string>(), *this);
    }

    for (unsigned i = 0; i < processes.size(); ++i) {
      processes[i]->PDDLPrint(os, 0, TokenStruct<std::string>(), *this);
    }
    for (unsigned i = 0; i < events.size(); ++i) {
      events[i]->PDDLPrint(os, 0, TokenStruct<std::string>(), *this);
    }

    if (constraints) {
      os << "( :constraints\n";
      constraints->PDDLPrint(os, 1, TokenStruct<std::string>(), *this);
      os << "\n)\n";
    }

    print_addtional_blocks(os);

    os << ")\n";
    return os;
  }

  virtual std::ostream & print_requirements(std::ostream & os) const
  {
    os << "( :requirements";
    if (!requirements.empty()) {  // As they were read
      for (const auto & requirement : requirements) {os << " :" << requirement;}
      os << " )\n";
      return os;
    }
    if (equality) {os << " :equality";}
    if (strips) {os << " :strips";}
    if (costs) {os << " :action-costs";}
    if (adl) {os << " :adl";}
    if (neg) {os << " :negative-preconditions";}
    if (condeffects) {os << " :conditional-effects";}
    if (typed) {os << " :typing";}
    if (temp) {os << " :durative-actions";}
    if (nondet) {os << " :non-deterministic";}
    if (universal) {os << " :universal-preconditions";}
    if (fluents) {os << " :fluents";}
    if (disj) {os << " :disjunctive-preconditions";}
    if (derivedpred) {os << " :derived-predicates";}
    if (existentialcond) {os << " :existential-preconditions";}
    os << " )\n";
    return os;
  }

  virtual std::ostream & print_addtional_blocks(std::ostream & os) const {return os;}

  virtual Condition * createCondition(Stringreader & f)
  {
    std::string s = f.getToken();

    if (s == "and") {return new And;}
    if (s == "exists") {return new Exists;}
    if (s == "forall") {return new Forall;}
    if (s == "imply") {return new Imply;}
    if (s == "assign") {return new Assign;}
    if (s == "increase") {return new Increase;}
    if (s == "decrease") {return new Decrease;}
    if (s == "scale-up") {return new ScaleUp;}
    if (s == "scale-down") {return new ScaleDown;}
    if (s == "not") {return new Not;}
    if (s == "oneof") {return new Oneof;}
    if (s == "or") {return new Or;}
    if (s == "when") {return new When;}
    // A modal operator takes conditions or times, never names: a predicate may be called so
    const auto mark = f.mark();
    f.next();
    if (s == "preference") {
      if (f.getChar() != '(') {
        f.getToken();
        f.next();
      }
      const bool cond = f.getChar() == '(';
      f.reset(mark);
      if (cond) {return new Preference;}
    } else if (s == "at" && f.getToken() == "end") {
      f.next();
      if (f.getChar() == '(') {return new Constraint("at end");}
    } else if (Constraint::isOperator(s)) {
      const char next = f.getChar();
      f.reset(mark);
      if (next == '(' || std::isdigit(static_cast<unsigned char>(next))) {
        return new Constraint(s);
      }
    }
    f.reset(mark);
    if (s == "=" || s == ">=" || s == ">" || s == "<=" || s == "<") {
      return new CompositeExpression(s);
    }

    int i = preds.index(s);
    if (i >= 0) {return new Ground(preds[i]);}

    f.tokenExit(s);

    return 0;
  }
};

}  // namespace pddl
}  // namespace parser
