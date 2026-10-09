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

// Malformed PDDL must be rejected, never crash the process (#416)

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gtest/gtest.h"

#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"

namespace
{

std::string read_file(const std::string & name)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_problem_expert").string();
  std::ifstream ifs(pkgpath + "/pddl/" + name);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

std::string repeat(const std::string & s, size_t n)
{
  std::string out;
  for (size_t i = 0; i < n; i++) {
    out += s;
  }
  return out;
}

const char * kValidProblem =
  "(define (problem p) (:domain simple)\n"
  "  (:objects leia - robot jack - person kitchen bedroom - room m1 - message)\n"
  "  (:init (robot_at leia kitchen) (person_at jack bedroom)\n"
  "         (= (room_distance kitchen bedroom) 10))\n"
  "  (:goal (and (robot_talk leia m1 jack))))\n";

std::string problem_with_goal(const std::string & goal)
{
  return
    "(define (problem p) (:domain simple)\n"
    "  (:objects leia - robot jack - person kitchen bedroom - room m1 - message)\n"
    "  (:init (robot_at leia kitchen))\n"
    "  (:goal " + goal + "))\n";
}

std::string problem_with_init(const std::string & init)
{
  return
    "(define (problem p) (:domain simple)\n"
    "  (:objects leia - robot jack - person kitchen bedroom - room m1 - message)\n"
    "  (:init " + init + ")\n"
    "  (:goal (and (robot_at leia bedroom))))\n";
}

}  // namespace

class MalformedProblemTest : public ::testing::TestWithParam<std::pair<std::string, std::string>>
{
protected:
  void SetUp() override
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
    problem_expert_ = std::make_shared<plansys2::ProblemExpert>(domain_expert_);
    ASSERT_TRUE(problem_expert_->addProblem(kValidProblem));
    baseline_ = problem_expert_->getProblem();
  }

  std::shared_ptr<plansys2::DomainExpert> domain_expert_;
  std::shared_ptr<plansys2::ProblemExpert> problem_expert_;
  std::string baseline_;
};

TEST_P(MalformedProblemTest, is_rejected_and_knowledge_is_untouched)
{
  ASSERT_FALSE(problem_expert_->addProblem(GetParam().second));
  ASSERT_EQ(problem_expert_->getProblem(), baseline_);

  // The expert is still usable afterwards
  ASSERT_TRUE(problem_expert_->addProblem(kValidProblem));
}

INSTANTIATE_TEST_SUITE_P(
  problem_expert, MalformedProblemTest,
  ::testing::Values(
    std::make_pair("empty", ""),
    std::make_pair("whitespace", " \n\t "),
    std::make_pair("garbage", "hello world"),
    std::make_pair("only_open_paren", "("),
    std::make_pair("only_close_paren", ")"),
    std::make_pair("open_parens", "(((("),
    std::make_pair("define_only", "(define)"),
    std::make_pair("problem_without_name", "(define (problem))"),
    std::make_pair("missing_domain", "(define (problem p))"),
    std::make_pair(
      "unbalanced_open",
      "(define (problem p) (:domain simple) (:objects leia - robot)"),
    std::make_pair(
      "unbalanced_close",
      "(define (problem p) (:domain simple))) )))"),
    std::make_pair(
      "deep_nesting",
      problem_with_goal(repeat("(and ", 3000) + "(robot_at leia kitchen)" + repeat(")", 3000))),
    std::make_pair(
      "unknown_type",
      "(define (problem p) (:domain simple) (:objects x - spaceship) (:init) (:goal (and)))"),
    std::make_pair(
      "wrong_domain",
      "(define (problem p) (:domain other) (:objects leia - robot) (:init) (:goal (and)))"),
    std::make_pair("unknown_predicate_in_init", problem_with_init("(flying leia)")),
    std::make_pair("wrong_arity_in_init", problem_with_init("(robot_at leia)")),
    std::make_pair("undeclared_object_in_init", problem_with_init("(robot_at leia nowhere)")),
    std::make_pair("number_as_argument", problem_with_init("(robot_at leia 3)")),
    std::make_pair("unknown_function", problem_with_init("(= (battery leia) 3)")),
    std::make_pair(
      "wrong_function_arity", problem_with_init("(= (room_distance kitchen) 3)")),
    std::make_pair("unknown_predicate_in_goal", problem_with_goal("(and (flying leia))")),
    std::make_pair("undeclared_object_in_goal", problem_with_goal("(and (robot_at leia mars))")),
    std::make_pair("empty_not_in_goal", problem_with_goal("(and (not))")),
    std::make_pair("unclosed_goal", problem_with_goal("(and (robot_at leia kitchen)"))
  ),
  [](const auto & info) {return info.param.first;});

// Every prefix of a valid problem is malformed; none of them may crash
TEST(problem_expert_malformed, truncated_problems)
{
  auto domain_expert =
    std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);
  const std::string valid = kValidProblem;

  for (size_t len = 0; len + 1 < valid.size(); len++) {
    SCOPED_TRACE("prefix length " + std::to_string(len));
    // A prefix may still parse when only trailing parentheses are missing
    (void)problem_expert.addProblem(valid.substr(0, len));
  }
  ASSERT_TRUE(problem_expert.addProblem(valid));
}

// Removing any single parenthesis unbalances the problem; none of them may crash
TEST(problem_expert_malformed, problems_missing_one_parenthesis)
{
  auto domain_expert =
    std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);
  const std::string valid = kValidProblem;

  for (size_t i = 0; i < valid.size(); i++) {
    if (valid[i] != '(' && valid[i] != ')') {
      continue;
    }
    SCOPED_TRACE("without char " + std::to_string(i));
    std::string broken = valid;
    broken.erase(i, 1);
    ASSERT_FALSE(problem_expert.addProblem(broken));
  }
  ASSERT_TRUE(problem_expert.addProblem(valid));
}

TEST(problem_expert_malformed, problem_without_goal_is_accepted)
{
  auto domain_expert =
    std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(
    problem_expert.addProblem(
      "(define (problem p) (:domain simple)\n"
      "  (:objects leia - robot kitchen - room)\n"
      "  (:init (robot_at leia kitchen)))\n"));

  ASSERT_EQ(problem_expert.getInstances().size(), 2u);
  ASSERT_EQ(problem_expert.getPredicates().size(), 1u);
  ASSERT_TRUE(problem_expert.getGoal().nodes.empty());

  // A goal can still be set afterwards
  ASSERT_TRUE(
    problem_expert.setGoal(parser::pddl::fromString("(and (robot_at leia kitchen))")));
  ASSERT_TRUE(problem_expert.isGoalSatisfied(problem_expert.getGoal()));
}

TEST(problem_expert_malformed, exists_with_bare_predicate_body)
{
  auto domain_expert =
    std::make_shared<plansys2::DomainExpert>(read_file("domain_simple.pddl"));
  plansys2::ProblemExpert problem_expert(domain_expert);
  ASSERT_TRUE(problem_expert.addProblem(kValidProblem));

  auto tree = parser::pddl::fromString("(exists (?r) (robot_at leia ?r))");
  ASSERT_FALSE(tree.nodes.empty());
  ASSERT_EQ(tree.nodes[0].node_type, plansys2_msgs::msg::Node::EXISTS);
  ASSERT_EQ(tree.nodes[1].node_type, plansys2_msgs::msg::Node::PREDICATE);
}

TEST(problem_expert_malformed, empty_and_inside_and)
{
  // An empty (and) child is dropped instead of being dereferenced
  auto tree = parser::pddl::fromString("(and (and) (robot_at leia kitchen))");
  ASSERT_FALSE(tree.nodes.empty());
  ASSERT_EQ(parser::pddl::toString(tree), "(and (robot_at leia kitchen))");
}

// Malformed domains: the domain expert must reject them, not crash
class MalformedDomainTest : public ::testing::TestWithParam<std::pair<std::string, std::string>>
{
};

TEST_P(MalformedDomainTest, is_rejected_and_domain_is_untouched)
{
  plansys2::DomainExpert domain_expert(read_file("domain_simple.pddl"));
  const std::string before = domain_expert.getDomain();

  ASSERT_FALSE(domain_expert.changeDomain(GetParam().second));
  ASSERT_EQ(domain_expert.getDomain(), before);
}

INSTANTIATE_TEST_SUITE_P(
  domain_expert, MalformedDomainTest,
  ::testing::Values(
    std::make_pair("garbage", "hello world"),
    std::make_pair("only_open_paren", "("),
    std::make_pair("unbalanced_open", "(define (domain d) (:requirements :strips)"),
    std::make_pair("unbalanced_close", "(define (domain d)))))"),
    std::make_pair(
      "no_predicates_section",
      "(define (domain d) (:requirements :strips :typing) (:types thing)\n"
      "  (:action a :parameters (?t - thing) :precondition (and) :effect (and)))"),
    std::make_pair(
      "unknown_function_in_effect",
      "(define (domain d) (:requirements :strips :typing :fluents) (:types thing)\n"
      "  (:predicates (p ?t - thing)) (:functions (f ?t - thing))\n"
      "  (:action a :parameters (?t - thing) :precondition (p ?t)\n"
      "    :effect (increase (g ?t) 1)))"),
    std::make_pair(
      "unclosed_either",
      "(define (domain d) (:requirements :strips :typing) (:types a b)\n"
      "  (:predicates (p ?x - (either a b))")
  ),
  [](const auto & info) {return info.param.first;});

TEST(domain_expert_malformed, truncated_domains)
{
  const std::string valid = read_file("domain_simple.pddl");
  plansys2::DomainExpert domain_expert(valid);
  const std::string before = domain_expert.getDomain();

  // Every 7th prefix keeps the test fast while still hitting every section
  for (size_t len = 0; len + 1 < valid.size(); len += 7) {
    SCOPED_TRACE("prefix length " + std::to_string(len));
    (void)domain_expert.changeDomain(valid.substr(0, len));
  }
  ASSERT_TRUE(domain_expert.changeDomain(valid));
  ASSERT_EQ(domain_expert.getDomain(), before);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
