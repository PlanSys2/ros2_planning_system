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
#include <sstream>
#include <string>

#include "plansys2_pddl_parser/Instance.hpp"

namespace parser
{
namespace pddl
{

Expression * TemporalAction::parseDuration(
  Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  return createExpression(f, ts, d);
}

namespace
{
// An effect of a durative action without at start or at end: a continuous one, with #t
bool isContinuousEffect(const std::string & token)
{
  return token == "increase" || token == "decrease";
}
}  // namespace

void TemporalAction::parseDurationConstraint(
  Stringreader & f, TokenStruct<std::string> & ts, Domain & d, const std::string & op)
{
  if (op != "=" && op != "<=" && op != ">=" && op != "<" && op != ">") {
    f.tokenExit(op);
  }
  f.next();
  f.assert_token("?duration");
  std::unique_ptr<Expression> expr(parseDuration(f, ts, d));
  if (!expr) {
    throw std::runtime_error("Missing value of the duration");
  }
  f.next();
  f.assert_token(")");
  durationConstraints.emplace_back(op, expr.release());
}

void TemporalAction::printCondition(
  std::ostream & s, const TokenStruct<std::string> & ts, const Domain & d, const std::string & t,
  And * a) const
{
  for (unsigned i = 0; a && i < a->conds.size(); ++i) {
    std::ostringstream cond;
    a->conds[i]->PDDLPrint(cond, 3, ts, d);
    const std::string text = cond.str();
    if (text.find('\n') == std::string::npos) {  // On one line
      s << "\t\t( " << t << " " << text.substr(text.find_first_not_of('\t')) << " )\n";
    } else {
      s << "\t\t( " << t << "\n" << text << "\n\t\t)\n";
    }
  }
}

void TemporalAction::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  (void)indent;
  (void)ts;
  s << "( :durative-action " << name << "\n";

  s << "  :parameters ";

  TokenStruct<std::string> astruct;

  printParams(0, s, astruct, d);

  s << "  :duration ";
  if (durationConstraints.size() > 1) {
    s << "( and ";
  }
  for (size_t i = 0; i < durationConstraints.size(); ++i) {
    s << (i > 0 ? " " : "") << "( " << durationConstraints[i].first << " ?duration ";
    durationConstraints[i].second->PDDLPrint(s, 0, astruct, d);
    s << " )";
  }
  if (durationConstraints.size() > 1) {
    s << " )";
  }
  if (durationConstraints.empty()) {
    s << "( = ?duration ";
    if (durationExpr) {
      durationExpr->PDDLPrint(s, 0, astruct, d);
    } else {
      s << "1";
    }
    s << " )";
  }
  s << "\n";

  s << "  :condition\n";
  s << "\t( and\n";
  printCondition(s, astruct, d, "at start", reinterpret_cast<And *>(pre));
  printCondition(s, astruct, d, "over all", pre_o);
  printCondition(s, astruct, d, "at end", pre_e);
  s << "\t)\n";

  s << "  :effect\n";
  s << "\t( and\n";
  printCondition(s, astruct, d, "at start", reinterpret_cast<And *>(eff));
  printCondition(s, astruct, d, "at end", eff_e);
  for (unsigned i = 0; eff_c && i < eff_c->conds.size(); ++i) {
    eff_c->conds[i]->PDDLPrint(s, 2, astruct, d);
    s << "\n";
  }
  s << "\t)\n";

  s << ")\n";
}

plansys2_msgs::msg::Node::SharedPtr TemporalAction::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  (void)tree;
  (void)d;
  (void)replace;
  throw UnsupportedConstruct("TemporalAction");
}

void TemporalAction::parseCondition(
  Stringreader & f, TokenStruct<std::string> & ts, Domain & d, And * a)
{
  f.next();
  f.assert_token("(");
  std::unique_ptr<Condition> c(d.createCondition(f));
  c->parse(f, ts, d);
  a->conds.push_back(c.release());
}

void TemporalAction::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  (void)ts;
  f.next();
  f.assert_token(":parameters");
  f.assert_token("(");

  TokenStruct<std::string> astruct = f.parseTypedList(true, d.types);
  assertUnique(astruct, "parameter of " + name);
  params = d.convertTypes(astruct.types);

  f.next();
  f.assert_token(":duration");
  f.assert_token("(");
  f.next();
  std::string op = f.getToken();
  if (op == "and") {
    for (f.next(); f.getChar() != ')'; f.next()) {
      f.assert_token("(");
      f.next();
      parseDurationConstraint(f, astruct, d, f.getToken());
    }
    ++f.c;
  } else {
    parseDurationConstraint(f, astruct, d, op);
  }
  if (durationConstraints.empty()) {
    throw std::runtime_error("Empty duration of " + name);
  }
  for (auto & constraint : durationConstraints) {
    if (constraint.first == "=" || constraint.first[0] == '<') {
      durationExpr = constraint.second;
      break;
    }
    durationExpr = constraint.second;
  }

  // :condition and :effect are both optional
  f.next();
  if (f.getChar() == ')') {
    ++f.c;
    return;
  }
  f.assert_token(":");
  std::string s = f.getToken();
  if (s == "condition") {
    pre = new And;
    pre_o = new And;
    pre_e = new And;
    f.next();
    f.assert_token("(");
    if (f.getChar() != ')') {
      s = f.getToken();
      if (s == "and") {
        for (f.next(); f.getChar() != ')'; f.next()) {
          f.assert_token("(");
          s = f.getToken();
          f.next();
          std::string t = f.getToken();

          if (s == "at" && t == "start") {
            parseCondition(f, astruct, d, reinterpret_cast<And *>(pre));
          } else if (s == "over" && t == "all") {
            parseCondition(f, astruct, d, pre_o);
          } else if (s == "at" && t == "end") {
            parseCondition(f, astruct, d, pre_e);
          } else {
            f.tokenExit(s + " " + t);
          }

          f.next();
          f.assert_token(")");
        }
        ++f.c;
      } else {
        f.next();
        std::string t = f.getToken();

        if (s == "at" && t == "start") {
          parseCondition(f, astruct, d, reinterpret_cast<And *>(pre));
        } else if (s == "over" && t == "all") {
          parseCondition(f, astruct, d, pre_o);
        } else if (s == "at" && t == "end") {
          parseCondition(f, astruct, d, pre_e);
        } else {
          f.tokenExit(s + " " + t);
        }

        f.next();
        f.assert_token(")");
      }
    } else {
      ++f.c;
    }

    f.next();
    if (f.getChar() == ')') {
      ++f.c;
      return;
    }
    f.assert_token(":");
    s = f.getToken();
  }
  if (s != "effect") {f.tokenExit(s);}

  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    eff = new And;
    eff_e = new And;
    eff_c = new And;

    // One timed effect, its '(' read: at start, at end, or continuous
    auto parseEffect = [&]() {
        const auto position = f.c;
        std::string s = f.getToken();
        if (isContinuousEffect(s)) {
          f.c = position;
          std::unique_ptr<Condition> c(d.createCondition(f));
          c->parse(f, astruct, d);
          eff_c->conds.push_back(c.release());
          return;
        }
        f.next();
        std::string t = f.getToken();

        if (s == "at" && t == "start") {
          parseCondition(f, astruct, d, reinterpret_cast<And *>(eff));
        } else if (s == "at" && t == "end") {
          parseCondition(f, astruct, d, eff_e);
        } else {
          f.tokenExit(s + " " + t);
        }

        // #t is the time a continuous effect acts: an instant has none
        std::ostringstream timed;
        reinterpret_cast<And *>(s == "at" && t == "start" ? eff : eff_e)->conds.back()->PDDLPrint(
          timed, 0, astruct, d);
        if (timed.str().find("#t") != std::string::npos) {
          throw std::runtime_error(
                  "#t in an effect at " + t + " of " + name +
                  ": continuous effects go without at start or at end");
        }

        f.next();
        f.assert_token(")");
      };

    const auto position = f.c;
    s = f.getToken();
    if (s == "and") {
      for (f.next(); f.getChar() != ')'; f.next()) {
        f.assert_token("(");
        parseEffect();
      }
      ++f.c;
    } else {
      f.c = position;
      parseEffect();
    }
  } else {
    ++f.c;
  }

  f.next();
  f.assert_token(")");
}

GroundVec TemporalAction::preconsStart() {return getGroundsFromCondition(pre, false);}

GroundVec TemporalAction::preconsOverall() {return getGroundsFromCondition(pre_o, false);}

GroundVec TemporalAction::preconsEnd() {return getGroundsFromCondition(pre_e, false);}

CondVec TemporalAction::endEffects() {return getSubconditionsFromCondition(eff_e);}

GroundVec TemporalAction::addEndEffects() {return getGroundsFromCondition(eff_e, false);}

GroundVec TemporalAction::deleteEndEffects() {return getGroundsFromCondition(eff_e, true);}

}  // namespace pddl
}  // namespace parser
