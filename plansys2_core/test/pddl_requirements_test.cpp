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

// What a task uses, seen from its syntax, so the planner gives it to solvers that plan with it

#include <filesystem>
#include <set>
#include <string>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"

using Set = std::set<std::string>;

namespace
{
// A domain with one action of these precondition and effect
std::string action(const std::string & precondition, const std::string & effect)
{
  return "(define (domain d) (:requirements :strips) (:predicates (p) (q ?x))"
         " (:action a :parameters (?x) :precondition " + precondition +
         " :effect " + effect + "))";
}
}  // namespace

TEST(pddl_requirements, a_plain_task_uses_nothing_special)
{
  EXPECT_EQ(plansys2::pddl_requirements(action("(p)", "(not (p))")), Set{});
  EXPECT_EQ(plansys2::pddl_requirements(action("(and (p) (q ?x))", "(and (q ?x))")), Set{});
  EXPECT_EQ(plansys2::pddl_requirements(""), Set{});
}

TEST(pddl_requirements, declared_but_unused_does_not_count)
{
  // :adl in 61 PlanSys2 domains: what POPF plans with must still reach it
  const auto domain =
    "(define (domain d) (:requirements :adl :typing :fluents :negative-preconditions"
    " :conditional-effects :derived-predicates :durative-actions :quantified-preconditions)"
    " (:predicates (p)) (:action a :parameters () :precondition (p) :effect (not (p))))";
  EXPECT_EQ(plansys2::pddl_requirements(domain), Set{});

  // What the syntax cannot show counts, and unknown ones, which a parser may reject
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:requirements :strips :action-costs :multi-agent))"),
    (Set{":action-costs", ":multi-agent"}));
}

TEST(pddl_requirements, conditions)
{
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(not (p))", "(p)")), Set{":negative-preconditions"});
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(and (p) (not (q ?x)))", "(p)")),
    Set{":negative-preconditions"});
  // Inequality is equality, and a negation in an effect is a delete
  EXPECT_EQ(plansys2::pddl_requirements(action("(not (= ?x ?x))", "(not (p))")), Set{});
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(or (p) (q ?x))", "(p)")),
    Set{":disjunctive-preconditions"});
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(imply (p) (q ?x))", "(p)")),
    Set{":disjunctive-preconditions"});
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(exists (?y) (q ?y))", "(p)")),
    Set{":existential-preconditions"});
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(forall (?y) (q ?y))", "(p)")),
    Set{":universal-preconditions"});

  // Negated goals too, but not negated facts elsewhere in the problem
  EXPECT_EQ(
    plansys2::pddl_requirements(
      action("(p)", "(p)"), "(define (problem x) (:domain d) (:init (p)) (:goal (not (p))))"),
    Set{":negative-preconditions"});
}

TEST(pddl_requirements, effects_derived_and_durations)
{
  EXPECT_EQ(
    plansys2::pddl_requirements(action("(p)", "(when (p) (q ?x))")),
    Set{":conditional-effects"});
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:predicates (p) (q)) (:derived (q) (p)))"),
    Set{":derived-predicates"});

  const auto durative =
    "(define (domain d) (:predicates (p)) (:functions (level))"
    " (:durative-action fill :parameters () :duration (and (>= ?duration 1) (<= ?duration 10))"
    "  :condition (at start (not (p)))"
    "  :effect (and (at start (p)) (at end (not (p))) (increase (level) (* #t 2)))))";
  EXPECT_EQ(
    plansys2::pddl_requirements(durative),
    (Set{":continuous-effects", ":duration-inequalities", ":durative-actions",
      ":negative-preconditions", ":numeric-fluents"}));

  // A fixed duration is no inequality
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:predicates (p)) (:durative-action m :parameters ()"
      " :duration (= ?duration 5) :condition (at start (p)) :effect (at end (p))))"),
    Set{":durative-actions"});
}

TEST(pddl_requirements, fluents)
{
  EXPECT_EQ(
    plansys2::pddl_requirements("(define (domain d) (:functions (fuel) (dist ?a ?b)))"),
    Set{":numeric-fluents"});
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:types room) (:functions (fuel ?r - room) - number))"),
    Set{":numeric-fluents"});
  // Typed parameters are not the type of the function
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:types room robot) (:functions (at ?r - robot) - room (fuel)))"),
    (Set{":numeric-fluents", ":object-fluents"}));
  // Only object fluents, two typed by one "- room"
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:types room robot)"
      " (:functions (at ?r - robot) (home ?r - robot) - room))"),
    Set{":object-fluents"});
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:functions (fuel) (load) - number (pos) - room))"),
    (Set{":numeric-fluents", ":object-fluents"}));
}

TEST(pddl_requirements, pddl3_and_pddl_plus)
{
  EXPECT_EQ(
    plansys2::pddl_requirements(
      action("(p)", "(p)"),
      "(define (problem x) (:domain d) (:init) (:goal (and (p) (preference g (q a))))"
      " (:constraints (always (p))))"),
    (Set{":constraints", ":preferences"}));
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:predicates (p)) (:functions (t))"
      " (:process heat :parameters () :precondition (p) :effect (increase (t) (* #t 1)))"
      " (:event stop :parameters () :precondition (>= (t) 5) :effect (not (p))))"),
    (Set{":continuous-effects", ":numeric-fluents", ":time"}));
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:predicates (p)) (:constraint safe :parameters ()"
      " :condition (p)))"),
    Set{":time"});
}

TEST(pddl_requirements, timed_initial_literals_only_in_the_init)
{
  const auto domain = action("(p)", "(p)");
  EXPECT_EQ(
    plansys2::pddl_requirements(
      domain, "(define (problem x) (:domain d) (:init (p) (at 10.5 (not (p)))) (:goal (p)))"),
    Set{":timed-initial-literals"});
  // A value at a time is a timed initial fluent
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "(define (domain d) (:functions (f)))",
      "(define (problem x) (:domain d) (:init (= (f) 0) (at 2 (= (f) 5))) (:goal (p)))"),
    (Set{":numeric-fluents", ":timed-initial-fluents"}));
  // A predicate called at, and at start/at end, are not
  EXPECT_EQ(
    plansys2::pddl_requirements(
      domain, "(define (problem x) (:domain d) (:init (at robot kitchen)) (:goal (p)))"),
    Set{});
}

TEST(pddl_requirements, comments_and_case)
{
  EXPECT_EQ(
    plansys2::pddl_requirements(
      "; (when (p) (q))\n(DEFINE (DOMAIN d) (:PREDICATES (p) (q)) ; (:derived (q) (p))\n"
      " (:ACTION a :PARAMETERS () :PRECONDITION (NOT (p)) :EFFECT (p)))"),
    Set{":negative-preconditions"});
}

TEST(pddl_requirements, missing)
{
  const Set needed = {":conditional-effects", ":negative-preconditions", ":strips"};
  EXPECT_EQ(
    plansys2::missing_requirements(needed, {":strips", ":typing"}),
    (std::vector<std::string>{":conditional-effects", ":negative-preconditions"}));
  EXPECT_TRUE(plansys2::missing_requirements(needed, plansys2::known_pddl_requirements()).empty());
  // A solver that does not say gets everything
  EXPECT_TRUE(plansys2::missing_requirements(needed, {}).empty());
  EXPECT_TRUE(plansys2::missing_requirements({}, {":strips"}).empty());

  // Whatever the syntax shows is a requirement a solver can declare
  for (const auto & requirement : plansys2::pddl_requirements(
      "(define (domain d) (:predicates (p) (q)) (:derived (q) (or (p) (exists (?x) (p))))"
      " (:durative-action m :parameters () :duration (<= ?duration 5)"
      "  :condition (at start (not (p))) :effect (and (when (p) (q)) (increase (f) (* #t 1)))))",
      "(define (problem x) (:domain d) (:init (at 1 (p))) (:goal (preference g (p)))"
      " (:constraints (always (p))))"))
  {
    EXPECT_TRUE(plansys2::known_pddl_requirements().count(requirement)) << requirement;
  }
}

// Every task of the bank uses what its line says, and its feature shows in it
TEST(pddl_requirements, the_bank)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  ASSERT_GE(cases.size(), 60u);
  for (const auto & task : cases) {
    EXPECT_EQ(plansys2::pddl_requirements(task.domain, task.problem), task.uses) << task.name;
    for (const auto & requirement : task.uses) {
      EXPECT_TRUE(plansys2::known_pddl_requirements().count(requirement)) << task.name;
    }
  }
}
