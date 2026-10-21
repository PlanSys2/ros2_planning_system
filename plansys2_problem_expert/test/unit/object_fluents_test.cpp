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

// Object fluents: functions whose value is an object, as (pos ?r) - room

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

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
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
const char * kDomain =
  "(define (domain warehouse) (:requirements :strips :typing :object-fluents :numeric-fluents)"
  " (:types place - object room dock - place robot box - object)"
  " (:constants charger - dock)"
  " (:predicates (moved ?r - robot) (connected ?a ?b - place))"
  " (:functions (pos ?r - robot) - place (holder ?b - box) - robot (battery ?r - robot))"
  " (:action move :parameters (?r - robot ?from ?to - place)"
  "  :precondition (and (= (pos ?r) ?from) (connected ?from ?to))"
  "  :effect (and (assign (pos ?r) ?to) (moved ?r))))";

const char * kProblem =
  "(define (problem p) (:domain warehouse)"
  " (:objects r1 r2 - robot kitchen hall - room crate - box)"
  " (:init (connected kitchen hall) (= (pos r1) kitchen) (= (pos r2) charger)"
  "  (= (battery r1) 50))"
  " (:goal (and (= (pos r1) hall))))";

struct Experts
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  std::shared_ptr<plansys2::ProblemExpert> problem;
  Experts()
  {
    domain = std::make_shared<plansys2::DomainExpert>(kDomain);
    problem = std::make_shared<plansys2::ProblemExpert>(domain);
  }
};

bool truth(
  const std::string & expression, std::vector<plansys2::Predicate> predicates,
  std::vector<plansys2::Function> functions)
{
  return plansys2::check(parser::pddl::fromString(expression), predicates, functions);
}

plansys2::Function object_fluent(const std::string & text)
{
  return parser::pddl::fromStringFunction(text);
}
}  // namespace

TEST(object_fluents, the_domain_tells_their_type)
{
  Experts e;
  ASSERT_TRUE(e.domain->getParseError().empty()) << e.domain->getParseError();
  EXPECT_TRUE(e.domain->getUnsupported().empty());
  EXPECT_EQ(e.domain->getFunction("pos")->object, "place");
  EXPECT_EQ(e.domain->getFunction("holder")->object, "robot");
  EXPECT_TRUE(e.domain->getFunction("battery")->object.empty());
}

TEST(object_fluents, an_object_of_their_type)
{
  Experts e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));
  auto pos = e.problem->getFunction("(pos r1)");
  ASSERT_TRUE(pos);
  EXPECT_EQ(pos->object, "kitchen");
  EXPECT_EQ(e.problem->getFunction("(pos r2)")->object, "charger");  // A constant of the domain
  EXPECT_DOUBLE_EQ(e.problem->getFunction("(battery r1)")->value, 50.0);

  // A subtype fits; another type, an unknown object or an object for a number do not
  EXPECT_TRUE(e.problem->updateFunction(object_fluent("(= (pos r1) hall)")));
  EXPECT_FALSE(e.problem->updateFunction(object_fluent("(= (pos r1) crate)")));
  EXPECT_FALSE(e.problem->updateFunction(object_fluent("(= (pos r1) garage)")));
  EXPECT_FALSE(e.problem->updateFunction(object_fluent("(= (battery r1) hall)")));
  EXPECT_TRUE(e.problem->addFunction(object_fluent("(= (holder crate) r2)")));
  EXPECT_FALSE(e.problem->addFunction(object_fluent("(= (holder crate) kitchen)")));
  EXPECT_EQ(e.problem->getFunction("(pos r1)")->object, "hall");

  // And in a problem: a wrong one refuses it all
  EXPECT_FALSE(
    e.problem->addProblem(
      "(define (problem p) (:domain warehouse) (:objects r1 - robot crate - box)"
      " (:init (= (pos r1) crate)) (:goal (and)))"));
}

TEST(object_fluents, written_in_the_problem_and_read_back)
{
  Experts e;
  ASSERT_TRUE(e.problem->addProblem(kProblem));
  const auto text = e.problem->getProblem();
  EXPECT_NE(text.find("( = ( pos r1 ) kitchen )"), std::string::npos) << text;
  EXPECT_NE(text.find("( = ( pos r2 ) charger )"), std::string::npos) << text;

  Experts again;
  ASSERT_TRUE(again.problem->addProblem(text)) << text;
  EXPECT_EQ(again.problem->getFunction("(pos r1)")->object, "kitchen");
  EXPECT_EQ(again.problem->getFunction("(pos r2)")->object, "charger");
  EXPECT_EQ(parser::pddl::toString(again.problem->getGoal()), "(and (= (pos r1) hall))");
}

TEST(object_fluents, compared_by_name)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions = {
    object_fluent("(= (pos r1) kitchen)"), object_fluent("(= (pos r2) kitchen)"),
    object_fluent("(= (pos r3) hall)"), parser::pddl::fromStringFunction("(= (battery r1) 5)")};

  EXPECT_TRUE(truth("(= (pos r1) kitchen)", predicates, functions));
  EXPECT_FALSE(truth("(= (pos r1) hall)", predicates, functions));
  EXPECT_TRUE(truth("(not (= (pos r1) hall))", predicates, functions));
  EXPECT_TRUE(truth("(= kitchen (pos r1))", predicates, functions));
  // Two fluents
  EXPECT_TRUE(truth("(= (pos r1) (pos r2))", predicates, functions));
  EXPECT_FALSE(truth("(= (pos r1) (pos r3))", predicates, functions));
  // One with no object yet is equal to none
  EXPECT_FALSE(truth("(= (pos r9) kitchen)", predicates, functions));
  EXPECT_TRUE(truth("(not (= (pos r9) kitchen))", predicates, functions));
  // An object and a number are not equal; numbers still compare as numbers
  EXPECT_FALSE(truth("(= (battery r1) kitchen)", predicates, functions));
  EXPECT_TRUE(truth("(= (battery r1) 5)", predicates, functions));
  EXPECT_TRUE(truth("(> (battery r1) 2)", predicates, functions));
  // Plain objects too, as an equality precondition
  EXPECT_TRUE(truth("(= kitchen kitchen)", predicates, functions));
  EXPECT_FALSE(truth("(= kitchen hall)", predicates, functions));

  // Bound as an action does it: a parameter with its object
  auto tree = parser::pddl::fromString("(= (pos r1) ?0)");
  tree.nodes[2].parameters[0].name = "kitchen";
  EXPECT_TRUE(plansys2::check(tree, predicates, functions));
}

TEST(object_fluents, assigned_reading_the_state_before)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions = {object_fluent("(= (pos r1) kitchen)")};

  ASSERT_TRUE(
    plansys2::apply(
      parser::pddl::fromString(
        "(and (assign (pos r1) hall) (when (= (pos r1) kitchen) (moved r1))"
        " (when (= (pos r1) hall) (stayed r1)))"),
      predicates, functions));
  ASSERT_EQ(functions.size(), 1u);
  EXPECT_EQ(functions[0].object, "hall");
  ASSERT_EQ(predicates.size(), 1u);
  EXPECT_EQ(predicates[0].name, "moved");

  // One with no object gets one
  ASSERT_TRUE(
    plansys2::apply(
      parser::pddl::fromString("(and (assign (pos r2) dock))"), predicates, functions));
  ASSERT_EQ(functions.size(), 2u);
  EXPECT_EQ(functions[1].object, "dock");

  // Swapping two: both read before
  functions = {object_fluent("(= (pos r1) kitchen)"), object_fluent("(= (pos r2) hall)")};
  auto swap = parser::pddl::fromString("(and (assign (pos r1) ?0) (assign (pos r2) ?1))");
  for (auto & node : swap.nodes) {
    if (node.node_type == plansys2_msgs::msg::Node::PARAMETER) {
      node.parameters[0].name = node.parameters[0].name == "?0" ? "hall" : "kitchen";
    }
  }
  ASSERT_TRUE(plansys2::apply(swap, predicates, functions));
  EXPECT_EQ(functions[0].object, "hall");
  EXPECT_EQ(functions[1].object, "kitchen");
}

// Through the problem expert: changes recorded, and undone exactly
TEST(object_fluents, applied_and_undone_through_the_client)
{
  {
    auto node = std::make_shared<plansys2::ProblemExpertNode>();
    const auto domain_file =
      (std::filesystem::temp_directory_path() / "object_fluents_domain.pddl").string();
    {
      std::ofstream out(domain_file);
      out << kDomain;
    }
    node->set_parameter({"model_file", domain_file});
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
    ASSERT_TRUE(client->addProblem(kProblem));

    std::vector<plansys2::FactChange> changes;
    ASSERT_TRUE(
      plansys2::apply(
        parser::pddl::fromString("(and (assign (pos r1) hall) (assign (holder crate) r1))"),
        client, changes));
    EXPECT_EQ(client->getFunction("(pos r1)")->object, "hall");
    EXPECT_EQ(client->getFunction("(holder crate)")->object, "r1");
    EXPECT_EQ(changes.size(), 2u);
    EXPECT_TRUE(client->isGoalSatisfied(client->getGoal()));

    ASSERT_TRUE(plansys2::undo(changes, client));
    EXPECT_EQ(client->getFunction("(pos r1)")->object, "kitchen");
    EXPECT_FALSE(client->getFunction("(holder crate)").has_value());
    EXPECT_FALSE(client->isGoalSatisfied(client->getGoal()));

    // Changed by someone else since: kept
    changes.clear();
    ASSERT_TRUE(
      plansys2::apply(parser::pddl::fromString("(and (assign (pos r1) hall))"), client, changes));
    ASSERT_TRUE(client->updateFunction(object_fluent("(= (pos r1) kitchen)")));
    std::vector<std::string> kept;
    ASSERT_TRUE(plansys2::undo(changes, client, &kept));
    EXPECT_EQ(kept, std::vector<std::string>{"(pos r1)"});
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
