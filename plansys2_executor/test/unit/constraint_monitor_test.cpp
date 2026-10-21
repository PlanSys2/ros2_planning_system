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

// The PDDL3 constraints over the states a plan goes through: when each one is violated, and
// when it is not, as states come and time passes

#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_executor/ConstraintMonitor.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using plansys2::ConstraintMonitor;

namespace
{
// A state: the facts that hold; a condition is one fact, or (not fact)
ConstraintMonitor::Holds state(const std::set<std::string> & facts)
{
  return [facts](const plansys2_msgs::msg::Tree & cond) {
           const auto text = parser::pddl::toString(cond);
           if (text.rfind("(not ", 0) == 0) {
             return !facts.count(text.substr(5, text.size() - 6));
           }
           return facts.count(text) > 0;
         };
}

ConstraintMonitor monitor(const std::string & constraints)
{
  return ConstraintMonitor(parser::pddl::fromString(constraints));
}

// Each state at its time: the first violation, or what finish says at the last time
std::string run(
  const std::string & constraints,
  const std::vector<std::pair<double, std::set<std::string>>> & states)
{
  auto m = monitor(constraints);
  for (const auto & [t, facts] : states) {
    if (auto violated = m.update(t, state(facts))) {
      return "at " + std::to_string(t) + ": " + *violated;
    }
  }
  if (auto violated = m.finish(states.back().first)) {
    return "at end: " + *violated;
  }
  return "ok";
}
}  // namespace

TEST(constraint_monitor, always)
{
  EXPECT_EQ(run("(always (p))", {{0, {"(p)"}}, {1, {"(p)", "(q)"}}, {2, {"(p)"}}}), "ok");
  EXPECT_EQ(
    run("(always (p))", {{0, {"(p)"}}, {1, {}}, {2, {"(p)"}}}),
    "at 1.000000: (always (p))");
  EXPECT_EQ(run("(always (p))", {{0, {}}}), "at 0.000000: (always (p))");
  EXPECT_EQ(run("(always (not (alarm)))", {{0, {}}, {3, {"(p)"}}}), "ok");
}

TEST(constraint_monitor, sometime_and_at_end)
{
  EXPECT_EQ(run("(sometime (p))", {{0, {}}, {1, {"(p)"}}, {2, {}}}), "ok");
  EXPECT_EQ(run("(sometime (p))", {{0, {}}, {1, {"(q)"}}}), "at end: (sometime (p))");
  // In the initial state counts
  EXPECT_EQ(run("(sometime (p))", {{0, {"(p)"}}, {1, {}}}), "ok");

  EXPECT_EQ(run("(at end (p))", {{0, {}}, {1, {"(p)"}}}), "ok");
  EXPECT_EQ(run("(at end (p))", {{0, {"(p)"}}, {1, {}}}), "at end: (at end (p))");
}

TEST(constraint_monitor, at_most_once)
{
  EXPECT_EQ(run("(at-most-once (p))", {{0, {}}, {1, {}}}), "ok");
  // One interval, however long
  EXPECT_EQ(run("(at-most-once (p))", {{0, {}}, {1, {"(p)"}}, {2, {"(p)"}}, {3, {}}}), "ok");
  EXPECT_EQ(
    run("(at-most-once (p))", {{0, {"(p)"}}, {1, {}}, {2, {"(p)"}}}),
    "at 2.000000: (at-most-once (p))");
  // The same state told twice is not a new interval
  EXPECT_EQ(run("(at-most-once (p))", {{0, {"(p)"}}, {0.5, {"(p)"}}, {1, {"(p)", "(q)"}}}), "ok");
}

TEST(constraint_monitor, within)
{
  EXPECT_EQ(run("(within 5 (p))", {{0, {}}, {4.9, {"(p)"}}}), "ok");
  EXPECT_EQ(run("(within 5 (p))", {{0, {}}, {5, {"(p)"}}}), "ok");  // Its bound included
  EXPECT_EQ(run("(within 5 (p))", {{0, {}}, {5.5, {"(p)"}}}), "at 5.500000: (within 5.000000 (p))");
  EXPECT_EQ(run("(within 5 (p))", {{0, {}}, {2, {"(q)"}}}), "at end: (within 5.000000 (p))");

  // Time passes with no new state: the deadline goes by in tick
  auto m = monitor("(within 5 (p))");
  ASSERT_FALSE(m.update(0, state({})));
  EXPECT_FALSE(m.tick(4.9));
  EXPECT_TRUE(m.tick(5.1));
}

TEST(constraint_monitor, hold_during_and_hold_after)
{
  // Held in [2, 4), as the state stays between states
  auto m = monitor("(hold-during 2 4 (p))");
  ASSERT_FALSE(m.update(0, state({})));
  EXPECT_FALSE(m.tick(1.9));
  ASSERT_FALSE(m.update(1.95, state({"(p)"})));
  EXPECT_FALSE(m.tick(3));
  ASSERT_FALSE(m.update(4, state({})));  // Over at 4
  EXPECT_FALSE(m.tick(6));
  EXPECT_FALSE(m.finish(6));

  auto broken = monitor("(hold-during 2 4 (p))");
  ASSERT_FALSE(broken.update(0, state({"(p)"})));
  EXPECT_TRUE(broken.update(3, state({})));

  // A plan over before the window: never held
  auto early = monitor("(hold-during 2 4 (p))");
  ASSERT_FALSE(early.update(0, state({})));
  EXPECT_TRUE(early.finish(1));

  auto after = monitor("(hold-after 3 (p))");
  ASSERT_FALSE(after.update(0, state({})));
  ASSERT_FALSE(after.update(2.5, state({"(p)"})));
  EXPECT_FALSE(after.tick(10));
  EXPECT_TRUE(after.update(11, state({})));

  auto late = monitor("(hold-after 3 (p))");
  ASSERT_FALSE(late.update(0, state({})));
  EXPECT_FALSE(late.tick(3));
  EXPECT_TRUE(late.tick(3.1));
}

TEST(constraint_monitor, sometime_after_and_sometime_before)
{
  const std::string after = "(sometime-after (alarm) (fixed))";
  EXPECT_EQ(run(after, {{0, {}}, {1, {"(alarm)"}}, {2, {"(fixed)"}}}), "ok");
  EXPECT_EQ(run(after, {{0, {}}, {1, {"(alarm)", "(fixed)"}}}), "ok");  // In the same state
  EXPECT_EQ(
    run(after, {{0, {}}, {1, {"(alarm)"}}, {2, {}}}),
    "at end: (sometime-after (alarm) (fixed))");
  // Fixed, then the alarm again
  EXPECT_EQ(
    run(after, {{0, {"(alarm)"}}, {1, {"(fixed)"}}, {2, {"(alarm)"}}}),
    "at end: (sometime-after (alarm) (fixed))");
  EXPECT_EQ(run(after, {{0, {}}, {1, {"(fixed)"}}}), "ok");  // Never needed

  const std::string before = "(sometime-before (delivered) (picked))";
  EXPECT_EQ(run(before, {{0, {}}, {1, {"(picked)"}}, {2, {"(delivered)"}}}), "ok");
  // Before, not in the same state
  EXPECT_EQ(
    run(before, {{0, {}}, {1, {"(picked)", "(delivered)"}}}),
    "at 1.000000: (sometime-before (delivered) (picked))");
  EXPECT_EQ(
    run(before, {{0, {"(delivered)"}}}),
    "at 0.000000: (sometime-before (delivered) (picked))");
  EXPECT_EQ(run(before, {{0, {}}, {1, {"(picked)"}}}), "ok");
}

TEST(constraint_monitor, always_within)
{
  const std::string c = "(always-within 3 (alarm) (fixed))";
  EXPECT_EQ(run(c, {{0, {}}, {1, {"(alarm)"}}, {3.5, {"(fixed)"}}}), "ok");
  EXPECT_EQ(
    run(c, {{0, {}}, {1, {"(alarm)"}}, {4.5, {"(fixed)"}}}),
    "at 4.500000: (always-within 3.000000 (alarm) (fixed))");
  EXPECT_EQ(
    run(c, {{0, {}}, {1, {"(alarm)"}}, {2, {}}}),
    "at end: (always-within 3.000000 (alarm) (fixed))");
  // Each alarm needs its own fix
  EXPECT_EQ(
    run(c, {{0, {"(alarm)"}}, {1, {"(fixed)"}}, {5, {"(alarm)"}}, {7.9, {"(fixed)"}}}), "ok");

  auto m = monitor(c);
  ASSERT_FALSE(m.update(0, state({"(alarm)"})));
  EXPECT_FALSE(m.tick(2.9));
  EXPECT_TRUE(m.tick(3.1));
}

TEST(constraint_monitor, several_and_preferences)
{
  // Every one is followed; preferences are not
  auto m = monitor(
    "(and (always (p)) (preference soft (always (q))) (sometime (r))"
    " (preference (at end (q))))");
  EXPECT_EQ(m.describe(), (std::vector<std::string>{"(always (p))", "(sometime (r))"}));
  ASSERT_FALSE(m.update(0, state({"(p)"})));
  ASSERT_FALSE(m.update(1, state({"(p)", "(r)"})));
  EXPECT_FALSE(m.finish(1));

  EXPECT_TRUE(monitor("(preference p (always (q)))").empty());
  EXPECT_TRUE(ConstraintMonitor(plansys2_msgs::msg::Tree()).empty());
}

TEST(constraint_monitor, from_the_domain_and_the_problem)
{
  const std::string domain =
    "(define (domain d) (:requirements :strips :typing :constraints :preferences)"
    " (:types room - object lab - room) (:predicates (visited ?r - room) (alarm))"
    " (:action visit :parameters (?r - room) :precondition (and) :effect (and (visited ?r)))"
    " (:constraints (always (not (alarm)))))";
  const std::string problem =
    "(define (problem p) (:domain d) (:objects hall - room lab1 - lab) (:init)"
    " (:goal (and (visited hall)))"
    " (:constraints (and (forall (?r - room) (sometime (visited ?r)))"
    "  (preference tidy (at end (visited lab1))))))";
  ConstraintMonitor m(domain, problem);
  // The forall over the rooms, and those of the subtypes
  EXPECT_EQ(
    m.describe(),
    (std::vector<std::string>{
    "(always (not (alarm)))", "(sometime (visited hall))", "(sometime (visited lab1))"}));

  ASSERT_FALSE(m.update(0, state({})));
  ASSERT_FALSE(m.update(1, state({"(visited hall)"})));
  EXPECT_EQ(m.finish(1).value_or(""), "(sometime (visited lab1))");

  // Text it cannot read: nothing to follow
  EXPECT_TRUE(ConstraintMonitor("(define (domain", problem).empty());
  // Those of the domain only
  EXPECT_EQ(
    ConstraintMonitor(
      domain, "(define (problem p) (:domain d) (:objects hall - room) (:init) (:goal (and)))")
    .describe().size(), 1u);
}
