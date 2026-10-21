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

// PDDL domain grammar: what is accepted, how it is read and how it is written back

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Humble has only the string form, which newer distros deprecate
#if __has_include("ament_index_cpp/get_package_share_path.hpp")
#include "ament_index_cpp/get_package_share_path.hpp"
static std::string share_dir(const std::string & package)
{
  return ament_index_cpp::get_package_share_path(package).string();
}
#else
#include "ament_index_cpp/get_package_share_directory.hpp"
static std::string share_dir(const std::string & package)
{
  return ament_index_cpp::get_package_share_directory(package);
}
#endif
#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/TemporalAction.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using parser::pddl::Domain;
using parser::pddl::toString;

namespace
{
// A domain with the given blocks after the requirements
std::string domainWith(const std::string & requirements, const std::string & blocks)
{
  return "(define (domain d) (:requirements " + requirements + ")" + blocks + ")";
}

const char * kTypes =
  "(:types robot room)"
  "(:predicates (at ?r - robot ?x - room) (free ?x - room) (charged ?r - robot))"
  "(:functions (battery ?r - robot) (distance ?a ?b - room))";

std::string preTree(const Domain & domain, const std::string & action)
{
  plansys2_msgs::msg::Tree tree;
  domain.actions.get(action)->pre->getTree(tree, domain);
  std::string error;
  EXPECT_TRUE(parser::pddl::validateTree(tree, error)) << error;
  return toString(tree);
}

std::string effTree(const Domain & domain, const std::string & action)
{
  plansys2_msgs::msg::Tree tree;
  domain.actions.get(action)->eff->getTree(tree, domain);
  std::string error;
  EXPECT_TRUE(parser::pddl::validateTree(tree, error)) << error;
  return toString(tree);
}

std::string print(const Domain & domain)
{
  std::ostringstream os;
  os << domain;
  return os.str();
}

std::string readFile(const std::string & name)
{
  const auto path =
    share_dir("plansys2_pddl_parser") + "/pddl/" + name;
  std::ifstream ifs(path);
  return std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
}
}  // namespace

TEST(Grammar, goal_variables_are_written_as_variables)
{
  Domain domain(domainWith(":strips :typing :existential-preconditions", kTypes +
    std::string(
      "(:action move :parameters (?r - robot ?a ?b - room)"
      " :precondition (at ?r ?a) :effect (and (at ?r ?b) (not (at ?r ?a))))")));
  parser::pddl::Instance problem(domain);
  problem.name = "p";
  problem.addObject("r1", "robot");
  for (const auto & room : {"kitchen", "bedroom", "hall", "garage", "bath"}) {
    problem.addObject(room, "room");
  }
  problem.addGoal("(and (exists (?x - room) (at r1 ?x)) (free kitchen))");

  std::ostringstream text;
  text << problem;
  EXPECT_NE(text.str().find("( exists ( ?x - room )"), std::string::npos) << text.str();
  EXPECT_NE(text.str().find("( at r1 ?x )"), std::string::npos) << text.str();
  EXPECT_NE(text.str().find("( free kitchen )"), std::string::npos) << text.str();

  // The text written is a problem of the same domain
  Domain again(print(domain));
  parser::pddl::Instance reread(again, text.str());
  std::ostringstream text2;
  text2 << reread;
  EXPECT_EQ(text2.str(), text.str());
}

TEST(Grammar, exists_variables_stay_in_their_scope)
{
  // ?x outside the exists is not its variable
  EXPECT_ANY_THROW(
    Domain(domainWith(":strips :typing :existential-preconditions", kTypes +
    std::string(
      "(:action a :parameters (?r - robot)"
      " :precondition (and (exists (?x - room) (at ?r ?x)) (free ?x)) :effect (charged ?r))"))));

  // Shadowing: ?b of the exists hides the parameter ?b, only inside the exists
  Domain domain(domainWith(":strips :typing :existential-preconditions", kTypes +
    std::string(
      "(:action a :parameters (?r - robot ?b - room)"
      " :precondition (and (exists (?a - robot ?b - room) (at ?a ?b)) (at ?r ?b))"
      " :effect (charged ?r))")));
  EXPECT_EQ(
    preTree(domain, "a"),
    "(and (exists (?2 - robot ?3 - room) (at ?2 ?3))(at ?0 ?1))");

  // Nested quantifiers number their variables after the outer ones; siblings reuse them
  Domain nested(domainWith(":strips :typing :quantified-preconditions", kTypes +
    std::string(
      "(:action a :parameters (?r - robot)"
      " :precondition (and (exists (?x - room) (forall (?y - room) (and (at ?r ?x) (free ?y))))"
      "  (exists (?z - room) (free ?z)))"
      " :effect (charged ?r))")));
  EXPECT_EQ(
    preTree(nested, "a"),
    "(and (exists (?1 - room) (forall (?2 - room) (and (at ?0 ?1)(free ?2))))"
    "(exists (?1 - room) (free ?1)))");

  // forall also hides an outer name
  Domain forall(domainWith(":strips :typing :universal-preconditions", kTypes +
    std::string(
      "(:action a :parameters (?r - robot ?x - room)"
      " :precondition (and (forall (?x - room) (free ?x)) (at ?r ?x))"
      " :effect (charged ?r))")));
  EXPECT_EQ(preTree(forall, "a"), "(and (forall (?2 - room) (free ?2))(at ?0 ?1))");

  // Repeated variables in one list
  EXPECT_ANY_THROW(
    Domain(domainWith(":strips :typing :existential-preconditions", kTypes +
    std::string(
      "(:action a :parameters (?r - robot)"
      " :precondition (exists (?x ?x - room) (at ?r ?x)) :effect (charged ?r))"))));
}

TEST(Grammar, or_with_any_number_of_arms)
{
  Domain domain(domainWith(":strips :typing :disjunctive-preconditions", kTypes +
    std::string(
      "(:action one :parameters (?r - robot ?x - room)"
      " :precondition (or (at ?r ?x)) :effect (charged ?r))"
      "(:action three :parameters (?r - robot ?x - room)"
      " :precondition (or (at ?r ?x) (free ?x) (charged ?r)) :effect (charged ?r))"
      "(:action nested :parameters (?r - robot ?x - room)"
      " :precondition (and (or (at ?r ?x) (not (free ?x))) (or (charged ?r) (free ?x)))"
      " :effect (charged ?r))")));
  EXPECT_EQ(preTree(domain, "one"), "(or (at ?0 ?1))");
  EXPECT_EQ(preTree(domain, "three"), "(or (at ?0 ?1)(free ?1)(charged ?0))");
  EXPECT_EQ(
    preTree(domain, "nested"), "(and (or (at ?0 ?1)(not (free ?1)))(or (charged ?0)(free ?1)))");

  for (const std::string pre : {"(or)", "(or () (at ?r ?x))", "(or (at ?r ?x) ())"}) {
    EXPECT_ANY_THROW(
      Domain(domainWith(":strips :typing :disjunctive-preconditions", kTypes +
      std::string(
        "(:action a :parameters (?r - robot ?x - room) :precondition " + pre +
        " :effect (charged ?r))")))) << pre;
  }
}

TEST(Grammar, arithmetic_with_any_number_of_operands)
{
  Domain domain(domainWith(":strips :typing :numeric-fluents", kTypes +
    std::string(
      "(:action a :parameters (?r - robot ?x ?y - room)"
      " :precondition (and (> (+ (battery ?r) 1 2) 0) (< (* (battery ?r) 2 3 4) 100)"
      "  (> (- (battery ?r)) -50) (> (battery ?r) (- 5)) (= (- (battery ?r) 1) 3)"
      "  (<= (/ (battery ?r) 2) (distance ?x ?y)))"
      " :effect (and (decrease (battery ?r) (+ (distance ?x ?y) 1 1))"
      "  (assign (battery ?r) (- (battery ?r)))))")));
  EXPECT_EQ(
    preTree(domain, "a"),
    "(and (> (+ (+ (battery ?0) 1.000000) 2.000000) 0.000000)"
    "(< (* (* (* (battery ?0) 2.000000) 3.000000) 4.000000) 100.000000)"
    "(> (- 0.000000 (battery ?0)) -50.000000)"
    "(> (battery ?0) (- 0.000000 5.000000))"
    "(= (- (battery ?0) 1.000000) 3.000000)"
    "(<= (/ (battery ?0) 2.000000) (distance ?1 ?2)))");
  EXPECT_EQ(
    effTree(domain, "a"),
    "(and (decrease (battery ?0) (+ (+ (distance ?1 ?2) 1.000000) 1.000000))"
    "(assign (battery ?0) (- 0.000000 (battery ?0))))");

  // Comparisons, - and / with a wrong number of operands
  for (const std::string pre : {
    "(> (battery ?r))", "(> (battery ?r) 1 2)", "(= (battery ?r) 1 2)",
    "(> (- (battery ?r) 1 2) 0)", "(> (/ (battery ?r)) 0)", "(> (/ (battery ?r) 1 2) 0)",
    "(> (+ (battery ?r)) 0)", "(> (*) 0)", "(> (-) 0)"})
  {
    EXPECT_ANY_THROW(
      Domain(domainWith(":strips :typing :numeric-fluents", kTypes +
      std::string(
        "(:action a :parameters (?r - robot) :precondition " + pre +
        " :effect (charged ?r))")))) << pre;
  }
}

TEST(Grammar, standard_requirements)
{
  for (const std::string requirement : {
    ":strips", ":typing", ":negative-preconditions", ":disjunctive-preconditions",
    ":equality", ":existential-preconditions", ":universal-preconditions",
    ":quantified-preconditions", ":conditional-effects", ":fluents", ":numeric-fluents",
    ":object-fluents", ":adl", ":durative-actions", ":duration-inequalities",
    ":continuous-effects", ":derived-predicates", ":timed-initial-literals", ":preferences",
    ":constraints", ":action-costs", ":action-cost", ":time"})
  {
    EXPECT_NO_THROW(Domain(domainWith(":typing " + requirement, kTypes))) << requirement;
  }
  EXPECT_ANY_THROW(Domain(domainWith(":typing :teleportation", kTypes)));

  // Written back as read
  Domain domain(domainWith(":strips :typing :numeric-fluents :action-costs", kTypes));
  EXPECT_NE(
    print(domain).find("( :requirements :strips :typing :numeric-fluents :action-costs )"),
    std::string::npos) << print(domain);
  EXPECT_TRUE(domain.costs);
  EXPECT_TRUE(domain.fluents);
  Domain quantified(domainWith(":typing :quantified-preconditions", kTypes));
  EXPECT_TRUE(quantified.existentialcond);
  EXPECT_TRUE(quantified.universal);
}

TEST(Grammar, durations_with_inequalities)
{
  const std::string conditions =
    " :condition (at start (free ?x)) :effect (at end (at ?r ?x)))";
  const std::vector<std::pair<std::string, std::string>> durations = {
    {"(= ?duration 5)", "( = ?duration 5 )"},
    {"(<= ?duration 5)", "( <= ?duration 5 )"},
    {"(>= ?duration (battery ?r))", "( >= ?duration ( battery ?robot0 ) )"},
    {"(< ?duration 10)", "( < ?duration 10 )"},
    {"(> ?duration 1.5)", "( > ?duration 1.5 )"},
    {"(and (>= ?duration 1) (<= ?duration 5))",
      "( and ( >= ?duration 1 ) ( <= ?duration 5 ) )"},
  };
  for (const auto & [duration, printed] : durations) {
    Domain domain(domainWith(":strips :typing :durative-actions :duration-inequalities", kTypes +
      std::string(
        "(:durative-action go :parameters (?r - robot ?x - room) :duration " + duration +
        conditions)));
    auto action = dynamic_cast<parser::pddl::TemporalAction *>(domain.actions.get("go"));
    ASSERT_NE(action, nullptr);
    ASSERT_NE(action->durationExpr, nullptr) << duration;
    EXPECT_NE(print(domain).find(":duration " + printed), std::string::npos) <<
      duration << "\n" << print(domain);
    EXPECT_EQ(print(Domain(print(domain))), print(domain)) << duration;
  }
  Domain bounds(domainWith(":strips :typing :durative-actions :duration-inequalities", kTypes +
    std::string(
      "(:durative-action go :parameters (?r - robot ?x - room)"
      " :duration (and (>= ?duration 1) (<= ?duration 5))" + conditions)));
  EXPECT_DOUBLE_EQ(bounds.actions.get("go")->duration(), 5.0);  // The upper bound

  for (const std::string duration : {
    "(!= ?duration 5)", "(= 5)", "(= ?d 5)", "(and)", "()", "(= ?duration)"})
  {
    EXPECT_ANY_THROW(
      Domain(domainWith(":strips :typing :durative-actions", kTypes +
      std::string(
        "(:durative-action go :parameters (?r - robot ?x - room) :duration " + duration +
        conditions)))) << duration;
  }
}

TEST(Grammar, actions_without_effect_or_precondition)
{
  Domain domain(domainWith(":strips :typing :durative-actions", kTypes +
    std::string(
      "(:action check :parameters (?r - robot ?x - room) :precondition (at ?r ?x))"
      "(:action noop :parameters (?r - robot))"
      "(:action charge :parameters (?r - robot) :effect (charged ?r))"
      "(:durative-action wait :parameters (?r - robot) :duration (= ?duration 2))"
      "(:durative-action look :parameters (?r - robot ?x - room) :duration (= ?duration 2)"
      " :condition (at start (at ?r ?x)))")));
  EXPECT_EQ(preTree(domain, "check"), "(at ?0 ?1)");
  EXPECT_EQ(domain.actions.get("check")->eff, nullptr);
  EXPECT_EQ(domain.actions.get("noop")->pre, nullptr);
  EXPECT_EQ(domain.actions.get("noop")->eff, nullptr);
  EXPECT_EQ(domain.actions.get("charge")->pre, nullptr);
  EXPECT_EQ(effTree(domain, "charge"), "(charged ?0)");
  EXPECT_EQ(domain.actions.size(), 5u);
  EXPECT_EQ(print(Domain(print(domain))), print(domain));

  // Something else after the precondition is still an error
  EXPECT_ANY_THROW(
    Domain(domainWith(":strips :typing", kTypes +
    std::string("(:action a :parameters (?r - robot) :precondition (charged ?r) :foo (x))"))));
}

TEST(Grammar, add_and_delete_effects)
{
  Domain domain(domainWith(":strips :typing :durative-actions", kTypes +
    std::string(
      "(:action both :parameters (?r - robot ?x - room)"
      " :effect (and (not (free ?x)) (at ?r ?x) (charged ?r)))"
      "(:action del :parameters (?x - room) :effect (not (free ?x)))"
      "(:action add :parameters (?x - room) :effect (free ?x))"
      "(:action none :parameters (?x - room))"
      "(:durative-action go :parameters (?r - robot ?x - room) :duration (= ?duration 1)"
      " :effect (and (at end (not (free ?x))) (at end (at ?r ?x))))")));
  EXPECT_EQ(domain.actions.get("both")->addEffects().size(), 2u);
  EXPECT_EQ(domain.actions.get("both")->deleteEffects().size(), 1u);
  EXPECT_EQ(domain.actions.get("del")->deleteEffects().size(), 1u);
  EXPECT_EQ(domain.actions.get("del")->addEffects().size(), 0u);
  EXPECT_EQ(domain.actions.get("add")->deleteEffects().size(), 0u);
  EXPECT_EQ(domain.actions.get("add")->addEffects().size(), 1u);
  EXPECT_EQ(domain.actions.get("none")->deleteEffects().size(), 0u);
  EXPECT_EQ(domain.actions.get("none")->addEffects().size(), 0u);
  auto go = dynamic_cast<parser::pddl::TemporalAction *>(domain.actions.get("go"));
  ASSERT_NE(go, nullptr);
  EXPECT_EQ(go->deleteEndEffects().size(), 1u);
  EXPECT_EQ(go->addEndEffects().size(), 1u);
}

TEST(Grammar, duplicates_are_rejected)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"predicate", "(:types robot) (:predicates (p ?r - robot) (p ?r - robot))"},
    {"function", "(:types robot) (:predicates (q)) (:functions (f ?r - robot) (f ?r - robot))"},
    {"action", "(:types robot) (:predicates (q))"
      "(:action a :parameters () :effect (q)) (:action a :parameters () :effect (q))"},
    {"durative action", "(:types robot) (:predicates (q))"
      "(:action a :parameters () :effect (q))"
      "(:durative-action a :parameters () :duration (= ?duration 1))"},
    {"action parameter", "(:types robot) (:predicates (p ?r - robot))"
      "(:action a :parameters (?r ?r - robot) :effect (p ?r))"},
    {"type of itself, listed twice", "(:types a b a - b)"},
    {"constant", "(:types robot) (:constants r1 r1 - robot)"},
    {"constant in two blocks", "(:types robot) (:constants r1 - robot) (:constants r1 - robot)"},
  };
  for (const auto & [what, blocks] : cases) {
    EXPECT_ANY_THROW(Domain(domainWith(":strips :typing :durative-actions :fluents", blocks))) <<
      what;
  }
  // As in IPC domains: a type listed again with another supertype (storage), repeated names in
  // a predicate declaration (logistics), and types without :typing
  EXPECT_NO_THROW(
    Domain(domainWith(":strips :typing",
    "(:types hoist surface place area - object storearea - area area crate - surface)"
    "(:predicates (clear ?s - storearea))")));
  EXPECT_NO_THROW(
    Domain(domainWith(":strips", "(:predicates (in ?obj ?obj)) (:action a :parameters (?x ?y)"
    " :effect (in ?x ?y))")));
  Domain untyped_requirement(domainWith(":strips",
    "(:types robot) (:predicates (p ?r - robot)) (:action a :parameters (?r - robot)"
    " :effect (p ?r))"));
  EXPECT_TRUE(untyped_requirement.typed);
  EXPECT_EQ(effTree(untyped_requirement, "a"), "(p ?0)");

  // Several :derived for the same predicate are a disjunction, not a duplicate
  EXPECT_NO_THROW(
    Domain(domainWith(":strips :typing :derived-predicates",
    "(:types robot) (:predicates (p ?r - robot) (q ?r - robot) (s ?r - robot))"
    "(:derived (s ?r - robot) (p ?r)) (:derived (s ?r - robot) (q ?r))")));
}

TEST(Grammar, cycles_of_types_are_rejected)
{
  for (const std::string types : {
    "(:types a - b b - a)", "(:types a - a)", "(:types a - b b - c c - a)",
    "(:types a - b b - c c - b)"})
  {
    EXPECT_ANY_THROW(Domain(domainWith(":strips :typing", types))) << types;
  }
  // A deep hierarchy is fine, and its constants are found
  Domain domain(domainWith(":strips :typing",
    "(:types a - b b - c c - d d) (:constants k - a) (:predicates (p ?x - d))"
    "(:action act :parameters () :effect (p k)))"));
  EXPECT_TRUE(domain.isConstant("k"));
  EXPECT_EQ(effTree(domain, "act"), "(p k)");
}

TEST(Grammar, scale_up_and_scale_down_effects)
{
  Domain domain(domainWith(":strips :typing :numeric-fluents", kTypes +
    std::string(
      "(:action a :parameters (?r - robot)"
      " :effect (and (scale-up (battery ?r) 2) (scale-down (battery ?r) 4)))")));
  EXPECT_EQ(
    effTree(domain, "a"),
    "(and (scale-up (battery ?0) 2.000000)(scale-down (battery ?0) 4.000000))");
  EXPECT_NE(print(domain).find("( scale-up ( battery ?robot0 ) 2 )"), std::string::npos) <<
    print(domain);
}

TEST(Grammar, constants_as_function_arguments)
{
  Domain domain(domainWith(":strips :typing :numeric-fluents",
    std::string("(:types robot room) (:constants dock - room)") +
    "(:predicates (at ?r - robot ?x - room))"
    "(:functions (distance ?a ?b - room) (battery ?r - robot))"
    "(:action go :parameters (?r - robot ?x - room)"
    " :precondition (> (distance dock ?x) 0)"
    " :effect (decrease (battery ?r) (distance ?x dock)))"));
  EXPECT_EQ(preTree(domain, "go"), "(> (distance dock ?1) 0.000000)");
  EXPECT_EQ(effTree(domain, "go"), "(decrease (battery ?0) (distance ?1 dock))");
  EXPECT_NE(print(domain).find("( distance dock ?room1 )"), std::string::npos) << print(domain);

  // A copy keeps the constants
  std::unique_ptr<parser::pddl::Condition> copy(domain.actions.get("go")->pre->copy(domain));
  plansys2_msgs::msg::Tree tree;
  copy->getTree(tree, domain);
  EXPECT_EQ(toString(tree), "(> (distance dock ?1) 0.000000)");
  std::ostringstream os;
  copy->PDDLPrint(os, 0, parser::pddl::TokenStruct<std::string>(), domain);
  EXPECT_NE(os.str().find("dock"), std::string::npos) << os.str();
}

TEST(Grammar, functions_without_parameters_and_parentheses)
{
  // PDDL 2.1 (zenotravel): (increase total-fuel-used ...)
  Domain domain(domainWith(":typing :fluents",
    "(:types plane) (:predicates (flying ?p - plane))"
    "(:functions (total-fuel-used) (burn ?p - plane))"
    "(:action fly :parameters (?p - plane)"
    " :precondition (< total-fuel-used 100)"
    " :effect (and (flying ?p) (increase total-fuel-used (burn ?p))))"));
  EXPECT_EQ(preTree(domain, "fly"), "(< (total-fuel-used) 100.000000)");
  EXPECT_EQ(effTree(domain, "fly"), "(and (flying ?0)(increase (total-fuel-used) (burn ?0)))");
  EXPECT_EQ(print(Domain(print(domain))), print(domain));

  // Only without parameters
  EXPECT_ANY_THROW(
    Domain(domainWith(":typing :fluents",
    "(:types plane) (:predicates (flying ?p - plane)) (:functions (burn ?p - plane))"
    "(:action fly :parameters (?p - plane) :effect (increase burn 1))")));
}

TEST(Grammar, continuous_effects_are_read)
{
  // Without at start or at end, alone or with others: they act while the action runs
  for (const std::string effect : {
    "(decrease (battery ?r) (* #t 0.5))",
    "(and (at end (charged ?r)) (decrease (battery ?r) (* #t 0.5)))",
    "(and (decrease (battery ?r) (* #t 0.5)) (increase (battery ?r) (* 2 #t)))"})
  {
    Domain domain(domainWith(":typing :fluents :durative-actions :continuous-effects", kTypes +
      std::string(
        "(:durative-action go :parameters (?r - robot) :duration (= ?duration 10)"
        " :effect " + effect + ")")));
    auto * go =
      dynamic_cast<parser::pddl::TemporalAction *>(domain.actions[domain.actions.index("go")]);
    ASSERT_NE(go, nullptr);
    ASSERT_NE(go->eff_c, nullptr) << effect;
    ASSERT_FALSE(go->eff_c->conds.empty()) << effect;
    plansys2_msgs::msg::Tree tree;
    go->eff_c->getTree(tree, domain);
    EXPECT_NE(parser::pddl::toString(tree).find("(decrease (battery ?0) (* #t 0.500000))"),
      std::string::npos)
      << effect << ": " << parser::pddl::toString(tree);
  }

  // #t at an instant is an error that says so
  try {
    Domain domain(domainWith(":typing :fluents :durative-actions :continuous-effects", kTypes +
      std::string(
        "(:durative-action go :parameters (?r - robot) :duration (= ?duration 10)"
        " :effect (at end (decrease (battery ?r) (* #t 0.5))))")));
    ADD_FAILURE() << "accepted #t at end";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("continuous effects go without"), std::string::npos)
      << e.what();
  }
}

TEST(Grammar, processes_and_events_are_read)
{
  Domain domain(domainWith(":typing :fluents :time",
    std::string(
      "(:types robot) (:predicates (heating ?r - robot)) (:functions (temperature ?r - robot))"
      "(:process heat :parameters (?r - robot) :precondition (heating ?r)"
      " :effect (increase (temperature ?r) (* #t 2)))"
      "(:event overheat :parameters (?r - robot) :precondition (>= (temperature ?r) 30)"
      " :effect (not (heating ?r)))")));
  ASSERT_EQ(domain.processes.size(), 1u);
  ASSERT_EQ(domain.events.size(), 1u);
  EXPECT_EQ(domain.processes[0]->keyword, "process");
  EXPECT_EQ(domain.events[0]->keyword, "event");

  // Written and read back
  std::ostringstream written;
  written << domain;
  EXPECT_NE(written.str().find("( :process heat"), std::string::npos) << written.str();
  EXPECT_NE(written.str().find("( :event overheat"), std::string::npos) << written.str();
  Domain again(written.str());
  EXPECT_EQ(again.processes.size(), 1u);
  EXPECT_EQ(again.events.size(), 1u);

  // A name twice is an error
  EXPECT_THROW(
    Domain(domainWith(":typing :fluents :time",
    std::string(
      "(:types robot) (:predicates (heating ?r - robot))"
      "(:event e :parameters (?r - robot) :precondition (heating ?r) :effect (not (heating ?r)))"
      "(:event e :parameters (?r - robot) :precondition (heating ?r) :effect (not (heating ?r)))"
    ))), std::runtime_error);
}

TEST(Grammar, actions_need_no_predicates)
{
  Domain domain(domainWith(":typing :numeric-fluents",
    "(:types robot) (:functions (battery ?r - robot))"
    "(:action charge :parameters (?r - robot) :precondition (< (battery ?r) 10)"
    " :effect (increase (battery ?r) 1))"));
  EXPECT_EQ(effTree(domain, "charge"), "(increase (battery ?0) 1.000000)");
}

TEST(Grammar, written_domains_and_problems_read_back_the_same)
{
  const std::vector<std::pair<std::string, std::string>> files = {
    {"dom1.pddl", "prob1.pddl"},
    {"dom2.pddl", "prob2.pddl"},
    {"domain_quantifiers.pddl", "problem_quantifiers.pddl"},
    {"suave_domain.pddl", "suave_problem.pddl"},
    {"suave_domain_extended_created.pddl", "suave_problem_extended_created.pddl"},
  };
  for (const auto & [domain_file, problem_file] : files) {
    Domain domain(readFile(domain_file));
    const std::string written = print(domain);
    Domain again(written);
    EXPECT_EQ(print(again), written) << domain_file;

    // The same conditions and effects
    for (unsigned i = 0; i < domain.actions.size(); ++i) {
      auto action = domain.actions[i];
      auto other = again.actions.get(action->name);
      auto compare = [&](parser::pddl::Condition * a, parser::pddl::Condition * b) {
          ASSERT_EQ(a == nullptr, b == nullptr) << action->name;
          if (!a) {return;}
          // ?duration has no tree: then neither of them
          plansys2_msgs::msg::Tree ta, tb;
          bool unsupported = false;
          try {
            a->getTree(ta, domain);
          } catch (const parser::pddl::UnsupportedConstruct &) {
            unsupported = true;
          }
          if (unsupported) {
            EXPECT_THROW(b->getTree(tb, again), parser::pddl::UnsupportedConstruct);
            return;
          }
          b->getTree(tb, again);
          EXPECT_EQ(toString(ta), toString(tb)) << domain_file << " " << action->name;
        };
      compare(action->pre, other->pre);
      compare(action->eff, other->eff);
      auto temporal = dynamic_cast<parser::pddl::TemporalAction *>(action);
      if (temporal) {
        auto other_temporal = dynamic_cast<parser::pddl::TemporalAction *>(other);
        ASSERT_NE(other_temporal, nullptr);
        compare(temporal->pre_o, other_temporal->pre_o);
        compare(temporal->pre_e, other_temporal->pre_e);
        compare(temporal->eff_e, other_temporal->eff_e);
      }
    }

    Domain problem_domain(readFile(domain_file));
    parser::pddl::Instance problem(problem_domain, readFile(problem_file));
    std::ostringstream problem_text;
    problem_text << problem;
    Domain problem_domain2(readFile(domain_file));
    parser::pddl::Instance problem2(problem_domain2, problem_text.str());
    std::ostringstream problem_text2;
    problem_text2 << problem2;
    EXPECT_EQ(problem_text2.str(), problem_text.str()) << problem_file;
  }
}
