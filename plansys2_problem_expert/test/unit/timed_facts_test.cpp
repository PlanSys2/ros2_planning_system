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

// Timed initial literals and fluents: kept with their time, written as what is still to come,
// and applied when their time comes

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
std::string domain_file()
{
  return plansys2::share_path("plansys2_problem_expert").string() +
         "/pddl/domain_when.pddl";
}

std::string read(const std::string & path)
{
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

const char * kProblem =
  "(define (problem p) (:domain cleaning)"
  " (:objects r1 - robot kitchen hall - room)"
  " (:init (robot_at r1 kitchen) (dirty hall) (= (battery r1) 10) (= (cleaned) 0)"
  "  (at 5 (not (dirty hall))) (at 2.5 (dirty kitchen)) (at 7 (= (battery r1) 100)))"
  " (:goal (and (robot_at r1 kitchen))))";

struct Expert
{
  double now = 100.0;
  std::shared_ptr<plansys2::DomainExpert> domain;
  std::shared_ptr<plansys2::ProblemExpert> problem;
  Expert()
  {
    domain = std::make_shared<plansys2::DomainExpert>(read(domain_file()));
    problem = std::make_shared<plansys2::ProblemExpert>(domain);
    problem->setClock([this]() {return now;});
  }
  bool holds(const std::string & p)
  {
    return problem->existPredicate(parser::pddl::fromStringPredicate(p));
  }
  double value(const std::string & f) {return problem->getFunction(f)->value;}
};
}  // namespace

TEST(timed_facts, loaded_in_time_order_from_now)
{
  Expert e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));
  const auto timed = e.problem->getTimedFacts();
  ASSERT_EQ(timed.size(), 3u);
  EXPECT_DOUBLE_EQ(timed[0].at, 102.5);
  EXPECT_FALSE(timed[0].negated);
  EXPECT_EQ(timed[0].fact.name, "dirty");
  EXPECT_DOUBLE_EQ(timed[1].at, 105.0);
  EXPECT_TRUE(timed[1].negated);
  EXPECT_DOUBLE_EQ(timed[2].at, 107.0);
  EXPECT_EQ(timed[2].fact.node_type, plansys2_msgs::msg::Node::FUNCTION);
  EXPECT_DOUBLE_EQ(timed[2].fact.value, 100.0);

  // Nothing has happened yet
  EXPECT_FALSE(e.holds("(dirty kitchen)"));
  EXPECT_TRUE(e.holds("(dirty hall)"));
}

TEST(timed_facts, the_problem_says_what_is_still_to_come)
{
  Expert e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));
  e.now = 103.0;
  ASSERT_TRUE(e.problem->applyTimedFacts());
  const auto text = e.problem->getProblem();

  // From now: 2 s and 4 s more; the one that came is in the state
  EXPECT_NE(text.find("( at 2.000000 ( not ( dirty hall ) ) )"), std::string::npos) << text;
  EXPECT_NE(text.find("( at 4.000000 ( = ( battery r1 ) 100"), std::string::npos) << text;
  EXPECT_EQ(text.find("( at 2.500000"), std::string::npos) << text;

  // Read back, it is the same
  parser::pddl::Domain domain(e.domain->getDomain());
  parser::pddl::Instance problem(domain);
  ASSERT_NO_THROW(problem.parse(text)) << text;
  EXPECT_EQ(problem.timedInit.size(), 2u);
}

TEST(timed_facts, applied_when_their_time_comes)
{
  Expert e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));

  e.now = 102.4;
  EXPECT_FALSE(e.problem->applyTimedFacts());
  EXPECT_FALSE(e.holds("(dirty kitchen)"));

  e.now = 102.5;
  EXPECT_TRUE(e.problem->applyTimedFacts());
  EXPECT_TRUE(e.holds("(dirty kitchen)"));
  EXPECT_TRUE(e.holds("(dirty hall)"));
  EXPECT_EQ(e.problem->getTimedFacts().size(), 2u);

  // Late: all that is due, in order
  e.now = 200.0;
  EXPECT_TRUE(e.problem->applyTimedFacts());
  EXPECT_FALSE(e.holds("(dirty hall)"));
  EXPECT_DOUBLE_EQ(e.value("(battery r1)"), 100.0);
  EXPECT_TRUE(e.problem->getTimedFacts().empty());
  EXPECT_FALSE(e.problem->applyTimedFacts());
}

TEST(timed_facts, the_same_fact_added_and_deleted)
{
  Expert e;
  ASSERT_TRUE(
    e.problem->addProblem(
      "(define (problem p) (:domain cleaning) (:objects r1 - robot hall - room)"
      " (:init (at 1 (dirty hall)) (at 3 (not (dirty hall))) (at 3 (dirty hall))) (:goal (and)))"));
  e.now = 101.0;
  e.problem->applyTimedFacts();
  EXPECT_TRUE(e.holds("(dirty hall)"));
  // At the same time, as written: deleted, then added
  e.now = 103.0;
  e.problem->applyTimedFacts();
  EXPECT_TRUE(e.holds("(dirty hall)"));
}

TEST(timed_facts, wrong_ones_are_refused)
{
  Expert e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));
  const auto before = e.problem->getTimedFacts().size();

  // Unknown predicate or object, a negated value, a time before the start: the whole problem
  // is refused and nothing changes
  for (const std::string init : {
    "(at 2 (flying r1))", "(at 2 (dirty garage))", "(at 2 (not (= (battery r1) 3)))",
    "(at -1 (dirty hall))"})
  {
    EXPECT_FALSE(
      e.problem->addProblem(
        "(define (problem p) (:domain cleaning) (:objects r1 - robot hall - room)"
        " (:init " + init + ") (:goal (and)))")) << init;
    EXPECT_EQ(e.problem->getTimedFacts().size(), before) << init;
  }

  // Directly too
  EXPECT_FALSE(
    e.problem->addTimedFact(parser::pddl::fromStringPredicate("(flying r1)"), 1.0));
  auto battery = parser::pddl::fromStringFunction("(= (battery r1) 3)");
  EXPECT_FALSE(e.problem->addTimedFact(battery, 1.0, true));
  EXPECT_TRUE(e.problem->addTimedFact(battery, 1.0));
  // A delay in the past is now
  EXPECT_TRUE(e.problem->addTimedFact(parser::pddl::fromStringPredicate("(clean hall)"), -4.0));
  EXPECT_TRUE(e.problem->applyTimedFacts());
  EXPECT_TRUE(e.holds("(clean hall)"));
}

TEST(timed_facts, a_predicate_called_at_is_not_one)
{
  Expert e;
  e.domain = std::make_shared<plansys2::DomainExpert>(
    "(define (domain d) (:requirements :strips :typing) (:types robot room)"
    " (:predicates (at ?r - robot ?l - room)))");
  e.problem = std::make_shared<plansys2::ProblemExpert>(e.domain);
  ASSERT_TRUE(
    e.problem->addProblem(
      "(define (problem p) (:domain d) (:objects r1 - robot hall - room)"
      " (:init (at r1 hall)) (:goal (and)))"));
  EXPECT_TRUE(e.problem->getTimedFacts().empty());
  EXPECT_TRUE(e.holds("(at r1 hall)"));
}

TEST(timed_facts, cleared_with_the_knowledge)
{
  Expert e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));
  ASSERT_TRUE(e.problem->clearKnowledge());
  EXPECT_TRUE(e.problem->getTimedFacts().empty());
}

// The node applies them by its clock, and tells it as any other change
TEST(timed_facts, the_node_applies_them_on_time)
{
  {
    auto node = std::make_shared<plansys2::ProblemExpertNode>();
    node->set_parameter({"model_file", domain_file()});
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
    const auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(
      client->addProblem(
        "(define (problem p) (:domain cleaning) (:objects r1 - robot hall - room)"
        " (:init (at 0.5 (dirty hall)) (at 0.5 (= (cleaned) 3))) (:goal (and)))"));
    const auto dirty = parser::pddl::fromStringPredicate("(dirty hall)");
    EXPECT_FALSE(client->existPredicate(dirty));
    EXPECT_NE(client->getProblem().find("( at 0."), std::string::npos);

    ASSERT_TRUE(plansys2::testing::wait_for([&]() {return client->existPredicate(dirty);}, 3s));
    const double took =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_GE(took, 0.45);
    EXPECT_LT(took, 1.5);
    EXPECT_DOUBLE_EQ(client->getFunction("(cleaned)")->value, 3.0);
    EXPECT_EQ(client->getProblem().find("( at "), std::string::npos);
  }
  plansys2::drain_ros(200ms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
