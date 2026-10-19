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

// Derived predicates: recursive ones to a fixpoint, and a negated one only once complete

#include <algorithm>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace
{
// free is written before blocked, which it negates: computed in that order, every room would be
// free, since nothing is blocked yet
const char * kDomain =
  "(define (domain rooms) (:requirements :strips :typing :derived-predicates"
  " :negative-preconditions :existential-preconditions)"
  " (:types room)"
  " (:predicates (obstacle ?r - room) (corridor ?a ?b - room) (blocked ?r - room)"
  "  (free ?r - room) (reachable ?a ?b - room) (open_route ?a ?b - room))"
  " (:derived (free ?r - room) (not (blocked ?r)))"
  " (:derived (blocked ?r - room) (obstacle ?r))"
  " (:derived (reachable ?a ?b - room) (corridor ?a ?b))"
  " (:derived (reachable ?a ?b - room)"
  "  (exists (?m - room) (and (corridor ?a ?m) (reachable ?m ?b))))"
  " (:derived (open_route ?a ?b - room) (and (reachable ?a ?b) (free ?b))))";

struct World
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  std::vector<plansys2::Instance> rooms;
  std::vector<plansys2::DerivedRule> rules;

  explicit World(const std::string & text = kDomain)
  {
    domain = std::make_shared<plansys2::DomainExpert>(text);
    for (const auto & name : {"a", "b", "c", "d"}) {
      rooms.push_back(plansys2::Instance(name, "room"));
    }
    std::vector<plansys2::Derived> derived;
    std::set<std::string> names;
    for (const auto & d : domain->getDerivedPredicates()) {
      if (names.insert(d.predicate.name).second) {
        for (const auto & rule : domain->getDerivedPredicate(d.predicate.name)) {
          derived.push_back(rule);
        }
      }
    }
    rules = plansys2::make_derived_rules(derived, rooms);
  }

  std::set<std::string> derive(const std::vector<std::string> & facts)
  {
    std::vector<plansys2::Predicate> state;
    for (const auto & fact : facts) {
      state.push_back(parser::pddl::fromStringPredicate(fact));
    }
    std::vector<plansys2::Function> functions;
    std::set<std::string> ret;
    for (const auto & p : plansys2::derive(rules, state, functions)) {
      ret.insert(parser::pddl::toString(p));
    }
    return ret;
  }
};
}  // namespace

TEST(derived, a_negated_one_once_it_is_complete)
{
  World w;
  const auto state = w.derive({"(obstacle b)"});
  EXPECT_TRUE(state.count("(blocked b)"));
  EXPECT_FALSE(state.count("(free b)"));
  for (const auto & room : {"a", "c", "d"}) {
    EXPECT_TRUE(state.count(std::string("(free ") + room + ")")) << room;
    EXPECT_FALSE(state.count(std::string("(blocked ") + room + ")")) << room;
  }
}

TEST(derived, recursive_to_a_fixpoint)
{
  World w;
  const auto state = w.derive({"(corridor a b)", "(corridor b c)", "(corridor c d)"});
  for (const auto & route : {"(reachable a b)", "(reachable a c)", "(reachable a d)",
      "(reachable b d)", "(reachable c d)"})
  {
    EXPECT_TRUE(state.count(route)) << route;
  }
  EXPECT_FALSE(state.count("(reachable d a)"));
  EXPECT_FALSE(state.count("(reachable b a)"));

  // A cycle of corridors ends too
  const auto cycle = w.derive({"(corridor a b)", "(corridor b a)"});
  EXPECT_TRUE(cycle.count("(reachable a a)"));
  EXPECT_TRUE(cycle.count("(reachable b b)"));
  EXPECT_FALSE(cycle.count("(reachable a c)"));
}

TEST(derived, one_reads_others_of_lower_strata)
{
  World w;
  // open_route needs reachable (recursive) and free (which negates blocked)
  const auto state = w.derive({"(corridor a b)", "(corridor b c)", "(obstacle c)"});
  EXPECT_TRUE(state.count("(open_route a b)"));
  EXPECT_FALSE(state.count("(open_route a c)"));
  EXPECT_FALSE(state.count("(open_route b c)"));
}

TEST(derived, what_was_derived_before_is_computed_again)
{
  World w;
  // Stale derived facts in the state are not kept: the obstacle is gone
  const auto state = w.derive({"(blocked b)", "(reachable d a)"});
  EXPECT_FALSE(state.count("(blocked b)"));
  EXPECT_FALSE(state.count("(reachable d a)"));
  EXPECT_TRUE(state.count("(free b)"));
}

TEST(derived, a_cycle_through_a_negation_is_computed_together)
{
  // p if not q, q if not p: no order makes sense; it still ends
  World w(
    "(define (domain loop) (:requirements :strips :typing :derived-predicates"
    " :negative-preconditions) (:types room)"
    " (:predicates (p ?r - room) (q ?r - room) (base ?r - room))"
    " (:derived (p ?r - room) (and (base ?r) (not (q ?r))))"
    " (:derived (q ?r - room) (and (base ?r) (not (p ?r)))))");
  const auto state = w.derive({"(base a)"});
  EXPECT_GE(state.count("(p a)") + state.count("(q a)"), 1u);
}

TEST(derived, the_problem_expert_gives_them)
{
  auto domain = std::make_shared<plansys2::DomainExpert>(kDomain);
  plansys2::ProblemExpert problem(domain);
  ASSERT_TRUE(
    problem.addProblem(
      "(define (problem p) (:domain rooms) (:objects a b c - room)"
      " (:init (corridor a b) (corridor b c) (obstacle c)) (:goal (and (open_route a b))))"));
  EXPECT_TRUE(problem.existPredicate(parser::pddl::fromStringPredicate("(reachable a c)")));
  EXPECT_TRUE(problem.existPredicate(parser::pddl::fromStringPredicate("(free a)")));
  EXPECT_FALSE(problem.existPredicate(parser::pddl::fromStringPredicate("(free c)")));
  EXPECT_TRUE(problem.isGoalSatisfied(problem.getGoal()));

  // The obstacle goes: what follows from it changes
  ASSERT_TRUE(problem.removePredicate(parser::pddl::fromStringPredicate("(obstacle c)")));
  EXPECT_TRUE(problem.existPredicate(parser::pddl::fromStringPredicate("(free c)")));
  EXPECT_TRUE(problem.existPredicate(parser::pddl::fromStringPredicate("(open_route a c)")));
  const auto all = problem.getPredicates();
  EXPECT_TRUE(
    std::any_of(
      all.begin(), all.end(), [](const auto & p) {
        return parser::pddl::toString(p) == "(open_route a c)";
      }));
}

// Reachability along a corridor of rooms: what it costs
TEST(derived, a_long_corridor_is_quick)
{
  for (int rooms : {20, 40}) {
    auto domain = std::make_shared<plansys2::DomainExpert>(kDomain);
    std::vector<plansys2::Instance> instances;
    std::vector<plansys2::Predicate> state;
    for (int i = 0; i < rooms; i++) {
      instances.push_back(plansys2::Instance("r" + std::to_string(i), "room"));
      if (i > 0) {
        state.push_back(
          parser::pddl::fromStringPredicate(
            "(corridor r" + std::to_string(i - 1) + " r" + std::to_string(i) + ")"));
      }
    }
    std::vector<plansys2::Derived> derived;
    for (const auto & name : {"reachable"}) {
      for (const auto & rule : domain->getDerivedPredicate(name)) {
        derived.push_back(rule);
      }
    }
    auto rules = plansys2::make_derived_rules(derived, instances);
    std::vector<plansys2::Function> functions;
    const auto start = std::chrono::steady_clock::now();
    const auto result = plansys2::derive(rules, state, functions);
    const double took =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_EQ(result.size(), state.size() + static_cast<size_t>(rooms * (rooms - 1) / 2));
    // Joined over the facts: 3.8 s for 40 rooms trying every pair of objects, 0.1 s for 20;
    // now 20 ms and 2 ms, and about 20 times that in a coverage build
    EXPECT_LT(took, rooms == 20 ? 0.1 : 2.0) << rooms << " rooms";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
