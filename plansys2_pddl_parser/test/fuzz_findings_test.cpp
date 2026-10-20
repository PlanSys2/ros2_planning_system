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

// Inputs that fuzzing found: each must be rejected at once, never hang nor crash

#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"

namespace
{

std::string domain_with(const std::string & blocks)
{
  return "(define (domain d) (:requirements :typing) " + blocks + ")";
}

const char * kDomain =
  "(define (domain d) (:requirements :typing) (:types robot room)"
  " (:predicates (at ?r - robot ?x - room)))";

std::string problem_with(const std::string & blocks)
{
  return "(define (problem p) (:domain d) " + blocks + " (:init) (:goal (and)))";
}

}  // namespace

// A ',', '{' or '}' in a typed list was read as an empty name forever, eating memory
TEST(FuzzFindings, typed_lists_with_what_is_not_a_name)
{
  for (const auto & bad : {",", "{", "}"}) {
    const std::string b(bad);
    SCOPED_TRACE(b);
    for (const auto & blocks : {
      "(:types a " + b + " b - object)",
      "(:types a - " + b + ")",
      "(:types a b - (either " + b + " object))",
      "(:types a) (:constants c " + b + " d - a)",
      "(:types a) (:predicates (p ?x " + b + " ?y - a))",
      "(:types a) (:predicates (p ?x - " + b + "))"})
    {
      SCOPED_TRACE(blocks);
      EXPECT_THROW(parser::pddl::Domain(domain_with(blocks)), std::runtime_error);
    }
  }
}

TEST(FuzzFindings, objects_with_what_is_not_a_name)
{
  parser::pddl::Domain domain(kDomain);
  for (const auto & blocks : {
    "(:objects r1 , robot a b - room)", "(:objects r1 { - robot)", "(:objects r1 - })",
    "(:objects r1 - robot , a - room)"})
  {
    SCOPED_TRACE(blocks);
    EXPECT_THROW(parser::pddl::Instance(domain, problem_with(blocks)), std::runtime_error);
  }
}

// Still read: the same lists, well written
TEST(FuzzFindings, well_written_lists_are_still_read)
{
  EXPECT_NO_THROW(parser::pddl::Domain(domain_with(
      "(:types a b - object c - (either a b)) (:constants k1 k2 - a)"
      " (:predicates (p ?x ?y - a ?z - b))")));
  parser::pddl::Domain robots(kDomain);
  EXPECT_NO_THROW(
    parser::pddl::Instance(robots, problem_with("(:objects r1 r2 - robot a b - room)")));
}

// A type made its own subtype, and then given another supertype: the check walked the
// supertypes, which no longer showed the cycle, and every walk of the subtypes overflowed
TEST(FuzzFindings, cycles_of_subtypes_hidden_by_a_later_supertype)
{
  for (const auto & blocks : {
    "(:constants i) (:types object - object object - e)", "(:types a - a a - b)",
    "(:types a - b b - a a - c)", "(:types a - b) (:types b - a)",
    "(:types a - b b - c) (:types c - a)"})
  {
    SCOPED_TRACE(blocks);
    EXPECT_THROW(parser::pddl::Domain(domain_with(blocks)), std::runtime_error);
  }
  // A type listed twice with the same supertype, as IPC storage does, is still read
  EXPECT_NO_THROW(parser::pddl::Domain(domain_with("(:types a - b a - b c)")));
}

// An either type with object as an alternative was hung from object, as every type without a
// supertype: object, the either type and object again, forever
TEST(FuzzFindings, either_with_object_is_not_a_cycle)
{
  for (const auto & blocks : {
    "(:types f - (either object))", "(:types a f - (either a object))",
    "(:types f g - (either object)) (:constants k - f)"})
  {
    SCOPED_TRACE(blocks);
    try {
      parser::pddl::Domain domain(domain_with(
          std::string(blocks) + " (:predicates (p ?x - f))"
          " (:action act :parameters (?x - f) :precondition (p ?x) :effect (not (p ?x)))"));
      EXPECT_FALSE(domain.isConstant("nothing"));  // Walks the types
    } catch (const std::runtime_error &) {
      // Rejected is fine too: never an endless walk of the types
    }
  }
}

// A precondition nested 100000 levels overflowed the stack of the recursive parsers: a domain
// or a problem deeper than kMaxNestingDepth is rejected, as expressions already were
TEST(FuzzFindings, deep_nesting_is_rejected)
{
  auto nested = [](int depth) {
      std::string text;
      for (int i = 0; i < depth; i++) {
        text += "(and ";
      }
      text += "(p ?x)";
      for (int i = 0; i < depth; i++) {
        text += ")";
      }
      return text;
    };
  auto domain = [&](int depth) {
      return "(define (domain d) (:predicates (p ?x)) (:action a :parameters (?x)"
             " :precondition " + nested(depth) + " :effect (p ?x)))";
    };
  EXPECT_NO_THROW(parser::pddl::Domain(domain(100)));
  for (int depth : {parser::pddl::kMaxNestingDepth, 100000}) {
    EXPECT_THROW(parser::pddl::Domain(domain(depth)), std::runtime_error) << depth;
  }

  parser::pddl::Domain robots(kDomain);
  std::string goal = "(at r1 a)";
  for (int i = 0; i < parser::pddl::kMaxNestingDepth + 10; i++) {
    goal = "(and " + goal + ")";
  }
  EXPECT_THROW(
    parser::pddl::Instance(
      robots, "(define (problem p) (:domain d) (:objects r1 - robot a - room) (:init)"
      " (:goal " + goal + "))"), std::runtime_error);
}

// A function without name, "()", was declared; a ',' in an expression then read as that
// function, without moving on, and the expression grew forever
TEST(FuzzFindings, declarations_without_name_and_empty_expressions)
{
  for (const auto & blocks : {
    "(:predicates (p ?x) ())", "(:functions (f ?x) ())", "(:action :parameters () :effect (and))",
    "(:predicates (p)) (:functions (f) ()) (:constraints (always (= , 0)))",
    "(:predicates (p)) (:functions (f)) (:constraints (always (= (+ , (f)) 0)))"})
  {
    SCOPED_TRACE(blocks);
    EXPECT_THROW(parser::pddl::Domain(domain_with(blocks)), std::runtime_error);
  }
}

// Found measuring big problems: each object added took as many copies of its type as there
// were objects (k objects, k^2 / 2 strings), and an object got the type of another
TEST(ScaleFindings, objects_added_keep_one_type_each)
{
  parser::pddl::Domain domain(kDomain);
  parser::pddl::Instance problem(domain);
  const auto before = problem.ts.size();
  for (int i = 0; i < 2000; i++) {
    problem.addObject("o" + std::to_string(i), i % 2 ? "robot" : "room");
  }
  ASSERT_EQ(problem.ts.size(), before + 2000u);
  ASSERT_EQ(problem.ts.types.size(), problem.ts.size());
  for (int i = 0; i < 2000; i++) {
    EXPECT_EQ(problem.ts.types[problem.ts.index("o" + std::to_string(i))],
      i % 2 ? "robot" : "room");
  }
}
