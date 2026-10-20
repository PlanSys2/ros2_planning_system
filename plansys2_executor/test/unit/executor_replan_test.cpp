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

// PDDL+: a plan replaced while a durative action with continuous effects runs. What goes on acts
// for the time of the new plan, under its new id; what is dropped stops acting

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;

namespace
{
const char * kDomain =
  "(define (domain charging) (:requirements :strips :typing :durative-actions"
  " :duration-inequalities :numeric-fluents :continuous-effects :constraints :time"
  " :negative-preconditions)"
  " (:types robot) (:predicates (ready ?r - robot) (beeped ?r - robot) (beeping ?r - robot)"
  "  (lit ?r - robot) (shown ?r - robot) (door_open) (passed ?r - robot) (heater_on ?r - robot)"
  "  (warm ?r - robot) (charging ?r - robot))"
  " (:functions (battery ?r - robot) (temperature ?r - robot))"
  // One charge at a time: two would add up their rates
  " (:durative-action recharge :parameters (?r - robot)"
  "  :duration (and (>= ?duration 0.5) (<= ?duration 10))"
  "  :condition (and (at start (not (charging ?r))) (over all (ready ?r)))"
  "  :effect (and (at start (charging ?r)) (at end (not (charging ?r)))"
  "   (increase (battery ?r) (* #t 5))))"
  " (:action beep :parameters (?r - robot) :precondition (and (ready ?r))"
  "  :effect (and (beeped ?r)))"
  " (:durative-action long_beep :parameters (?r - robot) :duration (= ?duration 0.5)"
  "  :condition (and (at start (ready ?r)))"
  "  :effect (and (at start (beeping ?r)) (at end (not (beeping ?r)))))"
  " (:durative-action light :parameters (?r - robot) :duration (= ?duration 1)"
  "  :condition (and (at start (ready ?r))) :effect (and (at end (lit ?r))))"
  " (:action show :parameters (?r - robot) :precondition (and (lit ?r))"
  "  :effect (and (shown ?r)))"
  " (:durative-action pass :parameters (?r - robot) :duration (= ?duration 0.5)"
  "  :condition (and (at start (door_open))) :effect (and (at end (passed ?r))))"
  // Perception measures the temperature: the heater would raise it, if the dynamics did
  " (:process heating :parameters (?r - robot) :precondition (and (heater_on ?r))"
  "  :effect (and (increase (temperature ?r) (* #t 10))))"
  " (:action wait_warm :parameters (?r - robot) :precondition (and (>= (temperature ?r) 25))"
  "  :effect (and (warm ?r))))";

const char * kProblem =
  "(define (problem p) (:domain charging) (:objects r1 - robot)"
  " (:init (ready r1) (= (battery r1) 0)) (:goal (and (ready r1))))";

// It lasts what the plan gives it
class TimedPerformer : public plansys2::ActionExecutorClient
{
public:
  explicit TimedPerformer(const std::string & action, double extra = 0.0)
  : ActionExecutorClient("replan_" + action), extra_(extra)
  {
    set_parameter({"action_name", action});
    set_parameter({"rate", 20.0});
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    started_ = now();
    return ActionExecutorClient::on_activate(state);
  }

private:
  void do_work() override
  {
    if ((now() - started_).seconds() >= get_planned_duration() + extra_) {
      finish(true, 1.0, "done");
    }
  }

  rclcpp::Time started_;
  double extra_;  // It takes this much more than planned
};

plansys2_msgs::msg::Plan plan(
  const std::vector<std::pair<std::string, double>> & items, std::vector<double> times = {})
{
  plansys2_msgs::msg::Plan ret;
  for (size_t i = 0; i < items.size(); i++) {
    const auto & [action, duration] = items[i];
    plansys2_msgs::msg::PlanItem item;
    item.time = i < times.size() ? times[i] : 0.0;
    item.action = action;
    item.duration = duration;
    ret.items.push_back(item);
  }
  return ret;
}

class ExecutorReplanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto file = (std::filesystem::temp_directory_path() / "replan_charging.pddl").string();
    {
      std::ofstream out(file);
      out << kDomain;
    }
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    executor_node_ = std::make_shared<plansys2::ExecutorNode>();
    planner_node_ = std::make_shared<plansys2::PlannerNode>();
    domain_node_->set_parameter({"model_file", file});
    problem_node_->set_parameter({"model_file", file});
    problem_node_->set_parameter(
      {"measured_functions", std::vector<std::string>{"temperature"}});
    // The light takes a second more than planned
    for (const auto & action : {"recharge", "beep", "long_beep", "show", "pass", "wait_warm"}) {
      performers_.push_back(std::make_shared<TimedPerformer>(action));
    }
    performers_.push_back(std::make_shared<TimedPerformer>("light", 1.0));
    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    exe_.add_node(executor_node_->get_node_base_interface());
    exe_.add_node(planner_node_->get_node_base_interface());
    for (auto & performer : performers_) {
      exe_.add_node(performer->get_node_base_interface());
    }
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
    for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
      domain_node_->trigger_transition(transition);
      problem_node_->trigger_transition(transition);
      executor_node_->trigger_transition(transition);
      planner_node_->trigger_transition(transition);
    }
    planner_node_->set_parameter({"KAIROS.anytime", false});
    for (auto & performer : performers_) {
      performer->trigger_transition(Transition::TRANSITION_CONFIGURE);
    }
    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    problem_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    executor_ = std::make_shared<plansys2::ExecutorClient>(client_node_);
    ASSERT_TRUE(problem_->addProblem(kProblem));
  }

  void TearDown() override
  {
    executor_.reset();
    spinner_.reset();
    performers_.clear();
    plansys2::drain_ros(200ms);
  }

  double battery() {return problem_->getFunction("(battery r1)")->value;}

  // Until the last plan ends; whether it succeeded
  bool wait_end()
  {
    plansys2::testing::wait_for([&]() {return !executor_->execute_and_check_plan();}, 20s);
    auto result = executor_->getResult();
    return result && result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  std::shared_ptr<plansys2::ExecutorNode> executor_node_;
  std::shared_ptr<plansys2::PlannerNode> planner_node_;
  std::vector<std::shared_ptr<TimedPerformer>> performers_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_;
  std::shared_ptr<plansys2::ExecutorClient> executor_;

  void problem(const std::string & init, const std::string & goal)
  {
    ASSERT_TRUE(problem_->clearKnowledge());
    ASSERT_TRUE(
      problem_->addProblem(
        "(define (problem p) (:domain charging) (:objects r1 - robot) (:init " + init +
        ") (:goal (and " + goal + ")))"));
  }
};
}  // namespace

// The recharge goes on, now for 1 s: it charges 1.5 s of the first plan and 1 s of the second,
// and then no more
TEST_F(ExecutorReplanTest, a_running_continuous_effect_goes_on_with_its_new_plan)
{
  ASSERT_TRUE(executor_->start_plan_execution(plan({{"(recharge r1)", 4.0}})));
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return battery() > 0.0;}, 5s));
  std::this_thread::sleep_for(1500ms);

  ASSERT_TRUE(executor_->start_plan_execution(plan({{"(recharge r1)", 1.0}})));
  ASSERT_TRUE(wait_end());
  const double charged = battery();
  EXPECT_GT(charged, 9.0);
  EXPECT_LT(charged, 17.0);  // The 4 s of the first plan would give 20

  // Nothing acts on it any more: not under its old id, not under the new one
  std::this_thread::sleep_for(1s);
  EXPECT_NEAR(battery(), charged, 0.01);
}

// A new plan without the recharge: it stops charging at once
TEST_F(ExecutorReplanTest, a_dropped_continuous_effect_stops)
{
  ASSERT_TRUE(executor_->start_plan_execution(plan({{"(recharge r1)", 4.0}})));
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return battery() > 0.0;}, 5s));
  std::this_thread::sleep_for(1s);

  ASSERT_TRUE(executor_->start_plan_execution(plan({{"(beep r1)", 0.0}})));
  ASSERT_TRUE(wait_end());
  const double charged = battery();
  EXPECT_LT(charged, 10.0);
  std::this_thread::sleep_for(1s);
  EXPECT_NEAR(battery(), charged, 0.01);
  EXPECT_TRUE(problem_->existPredicate(parser::pddl::fromStringPredicate("(beeped r1)")));
}

// Asked once a second, the client sees the end at once: the feedback of the whole plan, at
// 5 Hz, kept the result waiting behind it, one message each time it asked
TEST_F(ExecutorReplanTest, a_client_asking_slowly_sees_the_end)
{
  ASSERT_TRUE(executor_->start_plan_execution(plan({{"(recharge r1)", 4.0}})));
  const auto start = std::chrono::steady_clock::now();
  while (executor_->execute_and_check_plan()) {
    std::this_thread::sleep_for(1s);
    ASSERT_LT(std::chrono::steady_clock::now() - start, 20s);
  }
  const double took =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GE(took, 4.0);
  EXPECT_LT(took, 6.5);
  auto result = executor_->getResult();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->result, plansys2_msgs::action::ExecutePlan::Result::SUCCESS);
}

// Late, the rest of the plan goes later with it: what comes after keeps its planned distance.
// The light takes 2 s for 1; show, 1.5 s after the light starts, waits for it, and the recharge,
// planned 1 s after show, starts 1 s after show, not at once
TEST_F(ExecutorReplanTest, a_late_action_shifts_the_rest)
{
  executor_node_->set_parameter({"timed_execution", true});
  ASSERT_TRUE(
    executor_->start_plan_execution(
      plan({{"(light r1)", 1.0}, {"(show r1)", 0.0}, {"(recharge r1)", 0.5}}, {0.0, 1.5, 2.5})));
  ASSERT_TRUE(wait_end());
  auto result = executor_->getResult();
  ASSERT_TRUE(result.has_value());
  std::map<std::string, double> start;
  for (const auto & info : result->action_execution_status) {
    start[info.action] = rclcpp::Time(info.start_stamp).seconds();
  }
  ASSERT_EQ(start.size(), 3u);
  EXPECT_GE(start["show"] - start["light"], 1.9);  // After the light, which took 2 s
  EXPECT_NEAR(start["recharge"] - start["show"], 1.0, 0.3);
}

// A plan replaced while it runs goes on with what the constraints saw: the robot beeped once,
// a second beep is too many
TEST_F(ExecutorReplanTest, replaced_the_constraints_keep_their_history)
{
  ASSERT_TRUE(
    problem_->addProblem(
      "(define (problem p) (:domain charging) (:objects r1 - robot)"
      " (:init (ready r1) (= (battery r1) 0)) (:goal (and (ready r1)))"
      " (:constraints (at-most-once (beeping r1))))"));
  ASSERT_TRUE(
    executor_->start_plan_execution(
      plan({{"(long_beep r1)", 0.5}, {"(recharge r1)", 4.0}}, {0.0, 0.5})));
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return battery() > 0.0;}, 5s));

  // The recharge goes on; the beep again
  ASSERT_TRUE(
    executor_->start_plan_execution(
      plan({{"(recharge r1)", 1.0}, {"(long_beep r1)", 0.5}}, {0.0, 0.1})));
  plansys2::testing::wait_for([&]() {return !executor_->execute_and_check_plan();}, 20s);
  auto result = executor_->getResult();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->result, plansys2_msgs::action::ExecutePlan::Result::FAILURE);
}

// A plan replaced by Kairos while it runs: the goal grows, the plan from where the world is now
// takes over, with what it keeps running
TEST_F(ExecutorReplanTest, kairos_replans_while_it_runs)
{
  problem("(ready r1) (= (battery r1) 0)", "(>= (battery r1) 10)");
  plansys2::PlannerClient planner(client_node_);
  plansys2::DomainExpertClient domain(client_node_);
  auto first = planner.getPlan(domain.getDomain(), problem_->getProblem());
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(executor_->start_plan_execution(first.value()));
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return battery() > 1.0;}, 5s));

  ASSERT_TRUE(
    problem_->setGoal(parser::pddl::fromString("(and (>= (battery r1) 10) (beeped r1))")));
  // Now the robot is charging: from here no charge starts. From before the running one began,
  // the new plan has it again, and it goes on
  EXPECT_TRUE(problem_->existPredicate(parser::pddl::fromStringPredicate("(charging r1)")));
  auto replanning = executor_->get_replanning_problem();
  ASSERT_TRUE(replanning.has_value());
  EXPECT_EQ(replanning->find("charging r1"), std::string::npos) << *replanning;
  EXPECT_FALSE(planner.getPlan(domain.getDomain(), problem_->getProblem()).has_value());
  auto second = planner.getPlan(domain.getDomain(), replanning.value());
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(executor_->start_plan_execution(second.value()));
  ASSERT_TRUE(wait_end());
  EXPECT_TRUE(problem_->isGoalSatisfied(problem_->getGoal()));
  const double charged = battery();
  std::this_thread::sleep_for(1s);
  EXPECT_NEAR(battery(), charged, 0.01);  // Nothing charges any more
}

// Timed, a plan counts from when its problem was read: what the world does by its clock, as the
// door that closes at 2.5 s, does not move with the time the plan took to start. Its pass, at
// 2 s of the problem, is at 2 s of the problem, though the plan starts 1 s later
TEST_F(ExecutorReplanTest, timed_plans_count_from_when_their_problem_was_read)
{
  executor_node_->set_parameter({"timed_execution", true});
  executor_node_->set_parameter({"at_start_requirements_timeout", 2.0});
  problem("(ready r1) (door_open) (at 2.5 (not (door_open)))", "(passed r1)");
  const auto read = executor_node_->now();
  auto timed = plan({{"(pass r1)", 0.5}}, {2.0});
  timed.stamp = read;
  std::this_thread::sleep_for(1s);  // Planning

  ASSERT_TRUE(executor_->start_plan_execution(timed));
  ASSERT_TRUE(wait_end());
  auto result = executor_->getResult();
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->action_execution_status.size(), 1u);
  const auto & pass = result->action_execution_status[0];
  EXPECT_NEAR(
    (rclcpp::Time(pass.start_happening_stamp) - read).seconds(), 2.0, 0.3);
  EXPECT_TRUE(problem_->existPredicate(parser::pddl::fromStringPredicate("(passed r1)")));

  // Due before it started, it runs at once, and what comes after keeps its time
  problem("(ready r1) (door_open) (at 2.5 (not (door_open)))", "(passed r1)");
  const auto read_late = executor_node_->now();
  auto late = plan({{"(beep r1)", 0.0}, {"(pass r1)", 0.5}}, {0.2, 2.0});
  late.stamp = read_late;
  std::this_thread::sleep_for(1s);
  ASSERT_TRUE(executor_->start_plan_execution(late));
  ASSERT_TRUE(wait_end());
  result = executor_->getResult();
  ASSERT_TRUE(result.has_value());
  for (const auto & info : result->action_execution_status) {
    const double at = (rclcpp::Time(info.start_happening_stamp) - read_late).seconds();
    EXPECT_NEAR(at, info.action == "beep" ? 1.0 : 2.0, 0.3) << info.action;
  }
}

// What perception measures, the dynamics leave alone: the heater does not warm the robot in the
// problem expert, perception does, and the plan waits for it
TEST_F(ExecutorReplanTest, measured_functions_come_from_perception)
{
  problem("(ready r1) (heater_on r1) (= (battery r1) 0) (= (temperature r1) 20)", "(warm r1)");
  ASSERT_TRUE(executor_->start_plan_execution(plan({{"(wait_warm r1)", 0.0}})));
  std::this_thread::sleep_for(1s);
  EXPECT_DOUBLE_EQ(problem_->getFunction("(temperature r1)")->value, 20.0);  // Not integrated
  EXPECT_TRUE(executor_->execute_and_check_plan());  // It waits for it

  // Perception says it is warm now
  ASSERT_TRUE(
    problem_->updateFunction(parser::pddl::fromStringFunction("(= (temperature r1) 26)")));
  ASSERT_TRUE(wait_end());
  EXPECT_TRUE(problem_->existPredicate(parser::pddl::fromStringPredicate("(warm r1)")));
  EXPECT_DOUBLE_EQ(problem_->getFunction("(temperature r1)")->value, 26.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
