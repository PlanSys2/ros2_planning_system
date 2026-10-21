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

// PDDL+ in the problem expert: processes and the continuous effects of running actions act
// for each step, and events happen when their precondition holds

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <string>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
const char * kDomain =
  "(define (domain greenhouse) (:requirements :strips :typing :fluents :time)"
  " (:types room tank)"
  " (:predicates (heating ?r - room) (alarm ?r - room) (open ?t - tank) (overflowed ?t - tank)"
  "  (logged ?r - room))"
  " (:functions (temperature ?r - room) (level ?t - tank) (flow ?t - tank) (capacity ?t - tank))"
  " (:process warm :parameters (?r - room) :precondition (heating ?r)"
  "  :effect (increase (temperature ?r) (* #t 2)))"
  " (:process fill :parameters (?t - tank) :precondition (open ?t)"
  "  :effect (increase (level ?t) (* #t (flow ?t))))"
  " (:event overheat :parameters (?r - room)"
  "  :precondition (and (heating ?r) (>= (temperature ?r) 30))"
  "  :effect (and (not (heating ?r)) (alarm ?r)))"
  " (:event log_alarm :parameters (?r - room) :precondition (and (alarm ?r) (not (logged ?r)))"
  "  :effect (logged ?r))"
  " (:event overflow :parameters (?t - tank)"
  "  :precondition (and (open ?t) (>= (level ?t) (capacity ?t)))"
  "  :effect (and (not (open ?t)) (overflowed ?t))))";

const char * kProblem =
  "(define (problem p) (:domain greenhouse) (:objects kitchen lab - room t1 - tank)"
  " (:init (heating kitchen) (= (temperature kitchen) 20) (= (temperature lab) 20)"
  "  (open t1) (= (level t1) 0) (= (flow t1) 3) (= (capacity t1) 10))"
  " (:goal (and)))";

struct World
{
  std::shared_ptr<plansys2::DomainExpert> domain;
  std::shared_ptr<plansys2::ProblemExpert> problem;
  World()
  {
    domain = std::make_shared<plansys2::DomainExpert>(kDomain);
    problem = std::make_shared<plansys2::ProblemExpert>(domain);
    EXPECT_TRUE(domain->getParseError().empty()) << domain->getParseError();
    EXPECT_TRUE(problem->addProblem(kProblem));
  }
  double value(const std::string & f) {return problem->getFunction(f)->value;}
  bool holds(const std::string & p)
  {
    return problem->existPredicate(parser::pddl::fromStringPredicate(p));
  }
};
}  // namespace

TEST(dynamics, processes_act_while_their_precondition_holds)
{
  World w;
  EXPECT_TRUE(w.problem->stepDynamics(0.5));
  EXPECT_DOUBLE_EQ(w.value("(temperature kitchen)"), 21.0);
  EXPECT_DOUBLE_EQ(w.value("(temperature lab)"), 20.0);  // Not heating
  EXPECT_DOUBLE_EQ(w.value("(level t1)"), 1.5);  // Its rate is a function

  ASSERT_TRUE(w.problem->removePredicate(parser::pddl::fromStringPredicate("(heating kitchen)")));
  w.problem->stepDynamics(0.5);
  EXPECT_DOUBLE_EQ(w.value("(temperature kitchen)"), 21.0);
  EXPECT_DOUBLE_EQ(w.value("(level t1)"), 3.0);

  // No time, no change
  EXPECT_FALSE(
    w.problem->removePredicate(parser::pddl::fromStringPredicate("(open t1)")) &&
    w.problem->stepDynamics(0.0));
}

TEST(dynamics, events_happen_when_their_precondition_holds)
{
  World w;
  // 20 + 2 * 4.5 = 29: not yet; the tank reaches 10 at 3.33 s and closes
  w.problem->stepDynamics(4.5);
  EXPECT_DOUBLE_EQ(w.value("(temperature kitchen)"), 29.0);
  EXPECT_TRUE(w.holds("(heating kitchen)"));
  EXPECT_FALSE(w.holds("(open t1)"));
  EXPECT_TRUE(w.holds("(overflowed t1)"));
  EXPECT_DOUBLE_EQ(w.value("(level t1)"), 13.5);  // Integrated with the step it got

  // Over 30: the alarm, and the event it enables, in the same step
  w.problem->stepDynamics(1.0);
  EXPECT_FALSE(w.holds("(heating kitchen)"));
  EXPECT_TRUE(w.holds("(alarm kitchen)"));
  EXPECT_TRUE(w.holds("(logged kitchen)"));

  // The process stopped with it, and the closed tank stays
  w.problem->stepDynamics(1.0);
  EXPECT_DOUBLE_EQ(w.value("(temperature kitchen)"), 31.0);
  EXPECT_DOUBLE_EQ(w.value("(level t1)"), 13.5);
}

TEST(dynamics, the_continuous_effects_of_running_actions)
{
  World w;
  ASSERT_TRUE(w.problem->removePredicate(parser::pddl::fromStringPredicate("(heating kitchen)")));
  ASSERT_TRUE(w.problem->removePredicate(parser::pddl::fromStringPredicate("(open t1)")));
  auto cool = parser::pddl::fromString("(and (decrease (temperature lab) (* #t 4)))");
  ASSERT_TRUE(w.problem->setContinuousEffect("(cool lab):0", cool, true));
  EXPECT_EQ(w.problem->getContinuousEffects().size(), 1u);

  w.problem->stepDynamics(0.25);
  EXPECT_DOUBLE_EQ(w.value("(temperature lab)"), 19.0);

  // Two actions on the same function add up
  ASSERT_TRUE(
    w.problem->setContinuousEffect(
      "(cool lab):1", parser::pddl::fromString("(and (decrease (temperature lab) (* #t 1)))"),
      true));
  w.problem->stepDynamics(1.0);
  EXPECT_DOUBLE_EQ(w.value("(temperature lab)"), 14.0);

  // Ended: it acts no more
  ASSERT_TRUE(w.problem->setContinuousEffect("(cool lab):0", plansys2_msgs::msg::Tree(), false));
  ASSERT_TRUE(w.problem->setContinuousEffect("(cool lab):1", plansys2_msgs::msg::Tree(), false));
  EXPECT_TRUE(w.problem->getContinuousEffects().empty());
  EXPECT_FALSE(w.problem->stepDynamics(1.0));
  EXPECT_DOUBLE_EQ(w.value("(temperature lab)"), 14.0);

  // A malformed tree is refused
  plansys2_msgs::msg::Tree wrong;
  wrong.nodes.resize(1);
  wrong.nodes[0].node_type = plansys2_msgs::msg::Node::AND;
  wrong.nodes[0].children = {5};
  EXPECT_FALSE(w.problem->setContinuousEffect("x", wrong, true));
}

TEST(dynamics, measured_functions_are_left_to_perception)
{
  World w;
  w.problem->setMeasuredFunctions({"temperature"});
  w.problem->stepDynamics(1.0);
  EXPECT_DOUBLE_EQ(w.value("(temperature kitchen)"), 20.0);
  EXPECT_DOUBLE_EQ(w.value("(level t1)"), 3.0);  // Not measured: integrated

  // What perception says reaches the events
  ASSERT_TRUE(
    w.problem->updateFunction(parser::pddl::fromStringFunction("(= (temperature kitchen) 35)")));
  w.problem->stepDynamics(0.1);
  EXPECT_TRUE(w.holds("(alarm kitchen)"));
  EXPECT_DOUBLE_EQ(w.value("(temperature kitchen)"), 35.0);
}

TEST(dynamics, new_objects_get_their_processes)
{
  World w;
  ASSERT_TRUE(w.problem->addInstance(plansys2::Instance("cellar", "room")));
  ASSERT_TRUE(w.problem->addPredicate(parser::pddl::fromStringPredicate("(heating cellar)")));
  ASSERT_TRUE(
    w.problem->addFunction(parser::pddl::fromStringFunction("(= (temperature cellar) 10)")));
  w.problem->stepDynamics(1.0);
  EXPECT_DOUBLE_EQ(w.value("(temperature cellar)"), 12.0);
}

// The node steps them by its clock
TEST(dynamics, the_node_runs_them)
{
  {
    const auto domain_file =
      (std::filesystem::temp_directory_path() / "dynamics_greenhouse.pddl").string();
    {
      std::ofstream out(domain_file);
      out << kDomain;
    }
    auto node = std::make_shared<plansys2::ProblemExpertNode>();
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
    const auto start = std::chrono::steady_clock::now();

    // The tank overflows after 3.33 s at 3 per second
    ASSERT_TRUE(
      plansys2::testing::wait_for(
        [&]() {
          return client->existPredicate(parser::pddl::fromStringPredicate("(overflowed t1)"));
        }, 8s));
    const double took =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_GT(took, 3.0);
    EXPECT_LT(took, 4.5);
    EXPECT_NEAR(client->getFunction("(temperature kitchen)")->value, 20.0 + 2 * took, 0.5);

    // And the action effects through the client
    ASSERT_TRUE(
      client->setContinuousEffect(
        "(cool lab):0", parser::pddl::fromString("(and (decrease (temperature lab) (* #t 10)))"),
        true));
    ASSERT_TRUE(
      plansys2::testing::wait_for(
        [&]() {return client->getFunction("(temperature lab)")->value < 15.0;}, 3s));
    ASSERT_TRUE(client->setContinuousEffect("(cool lab):0", plansys2_msgs::msg::Tree(), false));
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
