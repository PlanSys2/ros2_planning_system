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

// PDDL3 in the problem expert: the constraints are kept and written for the planner; the
// preferences count for the planner, and hold when executing

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"

namespace
{
const char * kDomain =
  "(define (domain delivery) (:requirements :strips :typing :constraints :preferences)"
  " (:types robot room item)"
  " (:predicates (robot_at ?r - robot ?l - room) (item_at ?i - item ?l - room)"
  "  (carrying ?r - robot ?i - item) (alarm))"
  " (:action move :parameters (?r - robot ?a ?b - room)"
  "  :precondition (and (robot_at ?r ?a)) :effect (and (not (robot_at ?r ?a)) (robot_at ?r ?b))))";

const char * kObjects = "(:objects r1 - robot hall lab - room cup - item)";

std::string problem(const std::string & blocks)
{
  return std::string("(define (problem p) (:domain delivery) ") + kObjects +
         " (:init (robot_at r1 hall) (item_at cup hall))" + blocks + ")";
}

struct World
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  std::shared_ptr<plansys2::ProblemExpert> expert;
  World()
  {
    domain = std::make_shared<plansys2::DomainExpert>(kDomain);
    expert = std::make_shared<plansys2::ProblemExpert>(domain);
    EXPECT_TRUE(domain->getParseError().empty()) << domain->getParseError();
  }
  bool set(const std::string & text)
  {
    return expert->setConstraints(parser::pddl::fromString(text));
  }
  std::string constraints() {return parser::pddl::toString(expert->getConstraints());}
};
}  // namespace

TEST(problem_constraints, kept_and_written_for_the_planner)
{
  World w;
  ASSERT_TRUE(
    w.expert->addProblem(
      problem(
        " (:goal (and (robot_at r1 lab) (preference tidy (item_at cup lab))))"
        " (:constraints (and (always (not (alarm))) (at-most-once (robot_at r1 hall))"
        "  (preference safe (within 10 (robot_at r1 lab)))))"
        " (:metric minimize (+ (is-violated tidy) (* 2 (is-violated safe))))")));
  EXPECT_EQ(
    w.constraints(),
    "(and (always (not (alarm)))(at-most-once (robot_at r1 hall))"
    "(preference safe (within 10.000000 (robot_at r1 lab))))");
  EXPECT_EQ(w.expert->getPreferenceNames(), (std::set<std::string>{"tidy", "safe"}));

  // The problem the planner gets has them, and reads back the same
  const auto text = w.expert->getProblem();
  EXPECT_NE(text.find("( :constraints"), std::string::npos) << text;
  EXPECT_NE(text.find("( preference tidy"), std::string::npos) << text;
  EXPECT_NE(text.find("is-violated safe"), std::string::npos) << text;
  World again;
  ASSERT_TRUE(again.expert->addProblem(text)) << text;
  EXPECT_EQ(again.constraints(), w.constraints());
  EXPECT_EQ(
    parser::pddl::toString(again.expert->getGoal()), parser::pddl::toString(w.expert->getGoal()));
  EXPECT_EQ(
    parser::pddl::toString(again.expert->getMetric()),
    parser::pddl::toString(w.expert->getMetric()));
  EXPECT_EQ(again.expert->getProblem(), text);

  // Gone with the knowledge
  ASSERT_TRUE(w.expert->clearKnowledge());
  EXPECT_TRUE(w.expert->getConstraints().nodes.empty());
  EXPECT_EQ(w.expert->getProblem().find(":constraints"), std::string::npos);
}

TEST(problem_constraints, preferences_hold_when_executing)
{
  World w;
  ASSERT_TRUE(
    w.expert->addProblem(
      problem(" (:goal (and (robot_at r1 hall) (preference tidy (item_at cup lab))))")));
  // The cup is not in the lab: the goal holds anyway
  EXPECT_TRUE(w.expert->isGoalSatisfied(w.expert->getGoal()));

  // A hard part that does not hold still fails
  EXPECT_FALSE(
    w.expert->isGoalSatisfied(
      parser::pddl::fromString("(and (robot_at r1 lab) (preference tidy (item_at cup hall)))")));
  // Preferences only: it holds
  EXPECT_TRUE(
    w.expert->isGoalSatisfied(parser::pddl::fromString("(and (preference (robot_at r1 lab)))")));
}

TEST(problem_constraints, what_is_valid)
{
  World w;
  ASSERT_TRUE(w.expert->addProblem(problem(" (:goal (and (robot_at r1 lab)))")));

  for (const std::string good : {
    "(always (not (alarm)))",
    "(and (sometime (carrying r1 cup)) (hold-during 1 3 (robot_at r1 hall)))",
    "(forall (?l - room) (at-most-once (robot_at r1 ?l)))",
    "(preference p (sometime-before (robot_at r1 lab) (carrying r1 cup)))",
    "(and (at end (item_at cup lab)) (always-within 5 (alarm) (not (alarm))))",
  })
  {
    EXPECT_TRUE(w.set(good)) << good;
    EXPECT_EQ(w.constraints(), parser::pddl::toString(parser::pddl::fromString(good))) << good;
  }

  for (const std::string bad : {
    "(robot_at r1 hall)",    // No modal operator
    "(and (always (alarm)) (robot_at r1 hall))",
    "(always (sometime (alarm)))",    // A constraint of a constraint
    "(always (preference p (alarm)))",
    "(preference p (preference q (always (alarm))))",
    "(always (robot_at cup hall))",    // Wrong types
    "(always (flying r1))",    // Not in the domain
    "(sometime (robot_at r1 garden))",    // Not an instance
  })
  {
    EXPECT_FALSE(w.set(bad)) << bad;
  }
  EXPECT_FALSE(w.expert->setConstraints(plansys2_msgs::msg::Tree()));

  // A constraint goes in the constraints, not in the goal
  EXPECT_FALSE(w.expert->setGoal(parser::pddl::fromString("(and (always (robot_at r1 lab)))")));
  EXPECT_TRUE(
    w.expert->setGoal(parser::pddl::fromString("(and (preference g (robot_at r1 lab)))")));
}

TEST(problem_constraints, the_metric_reads_existing_preferences)
{
  World w;
  ASSERT_TRUE(
    w.expert->addProblem(
      problem(" (:goal (and (robot_at r1 lab) (preference tidy (item_at cup lab))))")));
  EXPECT_TRUE(w.expert->setMetric(plansys2::Metric("minimize (is-violated tidy)")));
  EXPECT_FALSE(w.expert->setMetric(plansys2::Metric("minimize (is-violated messy)")));

  ASSERT_TRUE(w.set("(preference safe (always (not (alarm))))"));
  EXPECT_TRUE(
    w.expert->setMetric(plansys2::Metric("minimize (+ (is-violated tidy) (is-violated safe))")));

  // A problem whose metric names no preference is refused whole
  World other;
  EXPECT_FALSE(
    other.expert->addProblem(
      problem(" (:goal (and (robot_at r1 lab))) (:metric minimize (is-violated tidy))")));
  EXPECT_TRUE(other.expert->getGoal().nodes.empty());
}

TEST(problem_constraints, a_removed_instance_takes_them_away)
{
  World w;
  ASSERT_TRUE(
    w.expert->addProblem(
      problem(
        " (:goal (and (robot_at r1 hall))) (:constraints (always (not (carrying r1 cup))))")));
  ASSERT_FALSE(w.expert->getConstraints().nodes.empty());

  // One of their instances: in full, as one less would allow what they forbid
  ASSERT_TRUE(w.expert->removeInstance(plansys2::Instance("cup", "item")));
  EXPECT_TRUE(w.expert->getConstraints().nodes.empty());

  // Another one leaves them
  ASSERT_TRUE(w.set("(always (robot_at r1 hall))"));
  ASSERT_TRUE(w.expert->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(w.expert->removeInstance(plansys2::Instance("kitchen", "room")));
  EXPECT_EQ(w.constraints(), "(always (robot_at r1 hall))");
}

TEST(problem_constraints, a_wrong_problem_leaves_nothing_of_it)
{
  World w;
  ASSERT_TRUE(
    w.expert->addProblem(
      problem(" (:goal (and (robot_at r1 lab))) (:constraints (sometime (robot_at r1 lab)))")));
  const auto before = w.expert->getProblem();

  // Its constraints are wrong: the whole problem is refused, the previous one stays
  EXPECT_FALSE(
    w.expert->addProblem(
      problem(" (:goal (and (robot_at r1 hall))) (:constraints (robot_at r1 hall))")));
  EXPECT_EQ(w.expert->getProblem(), before);

  // Without :constraints, those there are kept, as the goal
  ASSERT_TRUE(w.expert->addProblem(problem(" (:goal (and (robot_at r1 hall)))")));
  EXPECT_EQ(w.constraints(), "(sometime (robot_at r1 lab))");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
