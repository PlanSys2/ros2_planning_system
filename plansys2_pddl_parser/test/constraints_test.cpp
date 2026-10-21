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

// PDDL3: preferences, (is-violated) in the metric, and the modal constraints of a domain and a
// problem, read, written back, and as trees

#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using parser::pddl::Domain;
using parser::pddl::Instance;

namespace
{
const char * kDomain =
  "(define (domain d) (:requirements :strips :typing :preferences :constraints :fluents)"
  " (:types robot room)"
  " (:predicates (robot_at ?r - robot ?l - room) (alarm) (visited ?l - room))"
  " (:functions (battery ?r - robot))"
  " (:action move :parameters (?r - robot ?a ?b - room)"
  "  :precondition (and (robot_at ?r ?a) (preference charged (>= (battery ?r) 10)))"
  "  :effect (and (not (robot_at ?r ?a)) (robot_at ?r ?b))))";

std::string problemWith(const std::string & blocks)
{
  return "(define (problem p) (:domain d) (:objects r1 - robot hall lab - room)"
         " (:init (robot_at r1 hall)) (:goal (and (robot_at r1 lab)))" + blocks + ")";
}

template<typename T>
std::string print(const T & item)
{
  std::ostringstream out;
  out << item;
  return out.str();
}

std::string tree(const parser::pddl::Condition * cond, const Domain & domain)
{
  plansys2_msgs::msg::Tree t;
  cond->getTree(t, domain);
  std::string error;
  EXPECT_TRUE(parser::pddl::validateTree(t, error)) << error;
  return parser::pddl::toString(t);
}
}  // namespace

TEST(Constraints, every_operator_is_read_and_written_back)
{
  Domain domain(kDomain);
  Instance problem(domain, problemWith(
      "(:constraints (and"
      " (always (not (alarm))) (sometime (visited lab)) (at-most-once (robot_at r1 hall))"
      " (within 10 (robot_at r1 lab)) (hold-after 2.5 (visited hall))"
      " (hold-during 1 4 (robot_at r1 hall)) (sometime-before (visited lab) (visited hall))"
      " (sometime-after (alarm) (visited hall)) (always-within 3 (alarm) (visited hall))"
      " (at end (robot_at r1 lab))))"));
  ASSERT_NE(problem.constraints, nullptr);
  const auto text = tree(problem.constraints, domain);
  EXPECT_EQ(
    text,
    "(and (always (not (alarm)))(sometime (visited lab))(at-most-once (robot_at r1 hall))"
    "(within 10.000000 (robot_at r1 lab))(hold-after 2.500000 (visited hall))"
    "(hold-during 1.000000 4.000000 (robot_at r1 hall))"
    "(sometime-before (visited lab) (visited hall))(sometime-after (alarm) (visited hall))"
    "(always-within 3.000000 (alarm) (visited hall))(at end (robot_at r1 lab)))");

  // The written problem, read again, is the same; its tree too
  const auto written = print(problem);
  EXPECT_NE(written.find("( :constraints"), std::string::npos) << written;
  Instance again(domain, written);
  EXPECT_EQ(print(again), written);
  EXPECT_EQ(tree(again.constraints, domain), text);
  EXPECT_EQ(parser::pddl::toString(parser::pddl::fromString(text)), text);
}

TEST(Constraints, preferences_in_goals_constraints_and_preconditions)
{
  Domain domain(kDomain);
  Instance problem(domain,
    "(define (problem p) (:domain d) (:objects r1 - robot hall lab - room)"
    " (:init (robot_at r1 hall))"
    " (:goal (and (robot_at r1 lab) (preference tidy (visited hall)) (preference (visited lab))))"
    " (:constraints (preference safe (always (not (alarm)))))"
    " (:metric minimize (+ (* 3 (is-violated tidy)) (is-violated safe))))");
  EXPECT_EQ(
    tree(problem.goal, domain),
    "(and (robot_at r1 lab)(preference tidy (visited hall))(preference (visited lab)))");
  EXPECT_EQ(tree(problem.constraints, domain), "(preference safe (always (not (alarm))))");

  plansys2_msgs::msg::Tree metric;
  problem.metric_expr->getTree(metric, domain);
  EXPECT_EQ(
    parser::pddl::toString(metric), "(+ (* 3.000000 (is-violated tidy)) (is-violated safe))");
  EXPECT_EQ(metric.nodes[3].name, "is-violated");
  ASSERT_EQ(metric.nodes[3].parameters.size(), 1u);
  EXPECT_EQ(metric.nodes[3].parameters[0].name, "tidy");

  // In a precondition, with the parameters of its action
  plansys2_msgs::msg::Tree pre;
  domain.actions.get("move")->pre->getTree(pre, domain);
  EXPECT_EQ(
    parser::pddl::toString(pre),
    "(and (robot_at ?0 ?1)(preference charged (>= (battery ?0) 10.000000)))");

  const auto written = print(problem);
  Instance again(domain, written);
  EXPECT_EQ(print(again), written);
  EXPECT_NE(print(domain).find("( preference charged"), std::string::npos);
}

TEST(Constraints, of_the_domain)
{
  Domain domain(
    "(define (domain d) (:requirements :strips :typing :constraints) (:types room)"
    " (:predicates (alarm) (visited ?l - room))"
    " (:constraints (and (always (not (alarm))) (forall (?l - room) (sometime (visited ?l))))))");
  ASSERT_NE(domain.constraints, nullptr);
  EXPECT_EQ(
    tree(domain.constraints, domain),
    "(and (always (not (alarm)))(forall (?0 - room) (sometime (visited ?0))))");
  const auto written = print(domain);
  Domain again(written);
  EXPECT_EQ(print(again), written);

  // None, and empty
  EXPECT_EQ(Domain(kDomain).constraints, nullptr);
  EXPECT_EQ(
    Domain("(define (domain d) (:requirements :strips) (:predicates (p)) (:constraints))")
    .constraints, nullptr);
}

TEST(Constraints, read_over_line_breaks)
{
  // The text the parser writes: an operator alone in its line
  Domain domain(kDomain);
  Instance problem(domain, problemWith(
      "(:constraints\n(and\n\t(at-most-once\n\t\t(robot_at r1 hall)\n\t)\n\t(at\nend\n"
      "(robot_at r1 lab))\n\t(within\n\t\t7\n(visited lab))))"));
  EXPECT_EQ(
    tree(problem.constraints, domain),
    "(and (at-most-once (robot_at r1 hall))(at end (robot_at r1 lab))"
    "(within 7.000000 (visited lab)))");
}

TEST(Constraints, set_from_text)
{
  Domain domain(kDomain);
  Instance problem(domain, problemWith(""));
  EXPECT_EQ(problem.constraints, nullptr);
  problem.addConstraints("(sometime (visited lab))");
  EXPECT_EQ(tree(problem.constraints, domain), "(sometime (visited lab))");
  problem.addConstraints("(always (not (alarm)))");
  EXPECT_EQ(tree(problem.constraints, domain), "(always (not (alarm)))");
  problem.addConstraints("");
  EXPECT_EQ(problem.constraints, nullptr);
  EXPECT_EQ(print(problem).find(":constraints"), std::string::npos);
}

TEST(Constraints, predicates_called_as_operators_are_still_predicates)
{
  // at, always, within and preference as predicates: their arguments are names
  Domain domain(
    "(define (domain d) (:requirements :strips :typing :constraints) (:types robot room)"
    " (:predicates (at ?r - robot ?l - room) (always ?r - robot) (within ?a ?b - room)"
    "  (preference ?r - robot))"
    " (:action go :parameters (?r - robot ?a ?b - room)"
    "  :precondition (and (at ?r ?a) (always ?r) (within ?a ?b) (preference ?r))"
    "  :effect (and (not (at ?r ?a)) (at ?r ?b))))");
  plansys2_msgs::msg::Tree pre;
  domain.actions.get("go")->pre->getTree(pre, domain);
  EXPECT_EQ(
    parser::pddl::toString(pre), "(and (at ?0 ?1)(always ?0)(within ?1 ?2)(preference ?0))");

  // And with constraints on them; an object called end too
  Instance problem(domain,
    "(define (problem p) (:domain d) (:objects r1 - robot hall end - room)"
    " (:init (at r1 hall) (always r1)) (:goal (and (at r1 end)))"
    " (:constraints (and (always (always r1)) (at end (at r1 end)) (sometime (at r1 end)))))");
  EXPECT_EQ(
    tree(problem.constraints, domain),
    "(and (always (always r1))(at end (at r1 end))(sometime (at r1 end)))");
  EXPECT_EQ(tree(problem.goal, domain), "(and (at r1 end))");
}

TEST(Constraints, malformed_ones_are_errors)
{
  Domain domain(kDomain);
  for (const std::string bad : {
    "(:constraints (within (visited lab)))",    // No time
    "(:constraints (within -1 (visited lab)))",    // Before the start
    "(:constraints (hold-during 1 (visited lab)))",    // One time of two
    "(:constraints (sometime-before (visited lab)))",    // One condition of two
    "(:constraints (never (visited lab)))",    // Not an operator
    "(:constraints (always (visited lab))",    // Unbalanced
  })
  {
    EXPECT_ANY_THROW(Instance(domain, problemWith(bad))) << bad;
  }
}
