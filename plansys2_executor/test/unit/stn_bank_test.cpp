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

// The orders the STN of a plan of the bank has to keep, whatever the performers take: the
// executor runs actions as soon as their parents allow it

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/bt_builder_plugins/simple_bt_builder.hpp"
#include "plansys2_executor/bt_builder_plugins/stn_bt_builder.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"

using Transition = lifecycle_msgs::msg::Transition;
using plansys2::ActionType;

namespace
{

class TestableSTN : public plansys2::STNBTBuilder
{
public:
  using STNBTBuilder::build_stn;
  using STNBTBuilder::check_paths;
  using STNBTBuilder::get_tree;
  using STNBTBuilder::is_executable;
};

plansys2_msgs::msg::Plan make_plan(
  const std::vector<std::tuple<double, std::string, double>> & items)
{
  plansys2_msgs::msg::Plan plan;
  for (const auto & [time, action, duration] : items) {
    plansys2_msgs::msg::PlanItem item;
    item.time = time;
    item.action = action;
    item.duration = duration;
    plan.items.push_back(item);
  }
  return plan;
}

plansys2::Node::Ptr find(
  const plansys2::Graph::Ptr & graph, const std::string & expression, double time,
  ActionType type)
{
  for (const auto & node : graph->nodes) {
    if (node->action.expression == expression && node->action.type == type &&
      std::abs(node->action.time - time) < 1e-6)
    {
      return node;
    }
  }
  return nullptr;
}

// One task of the bank loaded in the experts
class STNBankTest : public ::testing::Test
{
protected:
  void load(const std::string & name)
  {
    const auto cases = parser::pddl::read_pddl_bank(
      plansys2::share_path("plansys2_pddl_parser") /
        "bank");
    const auto it = std::find_if(
      cases.begin(), cases.end(), [&](const auto & c) {return c.name == name;});
    ASSERT_NE(it, cases.end()) << name;
    load_files(it->domain_file.string(), it->problem);
  }

  void load_text(const std::string & domain, const std::string & problem)
  {
    const auto file = (std::filesystem::temp_directory_path() / "stn_bank_test.pddl").string();
    {
      std::ofstream out(file);
      out << domain;
    }
    load_files(file, problem);
  }

  void load_files(const std::string & domain_file, const std::string & problem)
  {
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    domain_node_->set_parameter({"model_file", domain_file});
    problem_node_->set_parameter({"model_file", domain_file});
    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
    for (auto node : {std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(domain_node_),
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(problem_node_)})
    {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
      node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }
    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    ASSERT_TRUE(problem_client->addProblem(problem));
    builder_ = std::make_shared<TestableSTN>();
    builder_->set_clients(
      std::make_shared<plansys2::DomainExpertClient>(client_node_), problem_client);
    builder_->initialize("", "", 3);
  }

  void TearDown() override
  {
    builder_.reset();
    spinner_.reset();
  }

  // b only after a: a path from a to b
  bool after(
    const plansys2::Graph::Ptr & graph, std::tuple<std::string, double, ActionType> a,
    std::tuple<std::string, double, ActionType> b)
  {
    auto from = find(graph, std::get<0>(a), std::get<1>(a), std::get<2>(a));
    auto to = find(graph, std::get<0>(b), std::get<1>(b), std::get<2>(b));
    EXPECT_TRUE(from && to);
    return from && to && builder_->check_paths(to, from);
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  rclcpp::Node::SharedPtr client_node_;  // The clients of the builder do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<TestableSTN> builder_;
};

}  // namespace

// A conditional effect, applied at the end of its action, gives what the plan says only after
// what makes its condition true
TEST_F(STNBankTest, a_conditional_effect_after_its_condition)
{
  load("conditional-effects/p2");
  auto graph = builder_->build_stn(
    make_plan({
    {0.0, "(restore_power tiago main_panel kitchen_button kitchen)", 0.0},
    {1.0, "(restore_power tiago main_panel lab_button kitchen)", 0.0},
    {2.0, "(press_button tiago kitchen_button kitchen_door kitchen)", 0.0},
    {3.0, "(move tiago kitchen_door kitchen corridor)", 0.0}}));
  EXPECT_TRUE(
    after(
      graph, {"(restore_power tiago main_panel kitchen_button kitchen)", 0.0, ActionType::END},
      {"(press_button tiago kitchen_button kitchen_door kitchen)", 2.0, ActionType::END}));
  EXPECT_TRUE(
    after(
      graph, {"(press_button tiago kitchen_button kitchen_door kitchen)", 2.0, ActionType::END},
      {"(move tiago kitchen_door kitchen corridor)", 3.0, ActionType::START}));
}

// A non-durative action that deletes what another one needs waits for it to end
TEST_F(STNBankTest, a_non_durative_threat_waits_for_the_end)
{
  load("negative-goals/p2");
  auto graph = builder_->build_stn(
    make_plan({
    {0.0, "(move r1 hall kitchen)", 0.0}, {1.0, "(clean r1 kitchen)", 0.0},
    {2.0, "(move r1 kitchen hall)", 0.0}}));
  EXPECT_TRUE(
    after(
      graph, {"(move r1 hall kitchen)", 0.0, ActionType::END},
      {"(clean r1 kitchen)", 1.0, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(move r1 hall kitchen)", 0.0, ActionType::END},
      {"(move r1 kitchen hall)", 2.0, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(clean r1 kitchen)", 1.0, ActionType::END},
      {"(move r1 kitchen hall)", 2.0, ActionType::START}));
}

// The door is held open until the robot that passes through it is on the other side
TEST_F(STNBankTest, required_concurrency_ends_in_order)
{
  load("required-concurrency/p2");
  auto graph = builder_->build_stn(
    make_plan({
    {0.0, "(hold_door r1 office_door corridor office)", 10.0},
    {8.002, "(hold_door r3 lab_door office lab)", 10.0},
    {10.001, "(hold_door r2 office_door corridor office)", 10.0},
    {10.001, "(pass_door r1 office_door corridor office)", 4.0},
    {14.002, "(pass_door r1 lab_door office lab)", 4.0},
    {20.002, "(hold_door r3 office_door office corridor)", 10.0},
    {20.002, "(pass_door r2 office_door corridor office)", 4.0},
    {30.003, "(hold_door r2 office_door office corridor)", 10.0},
    {30.003, "(pass_door r3 office_door office corridor)", 4.0}}));
  // Held open before it passes, closed after it has passed
  EXPECT_TRUE(
    after(
      graph, {"(hold_door r2 office_door corridor office)", 10.001, ActionType::START},
      {"(pass_door r1 office_door corridor office)", 10.001, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(pass_door r1 office_door corridor office)", 10.001, ActionType::END},
      {"(hold_door r2 office_door corridor office)", 10.001, ActionType::END}));
  EXPECT_TRUE(
    after(
      graph, {"(pass_door r1 lab_door office lab)", 14.002, ActionType::END},
      {"(hold_door r3 lab_door office lab)", 8.002, ActionType::END}));
  // The first hold closes the door before the second one opens it: the pass waits for that one
  EXPECT_TRUE(
    after(
      graph, {"(hold_door r1 office_door corridor office)", 0.0, ActionType::END},
      {"(hold_door r2 office_door corridor office)", 10.001, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(pass_door r2 office_door corridor office)", 20.002, ActionType::END},
      {"(hold_door r3 office_door office corridor)", 20.002, ActionType::END}));
}

// Each action is executed once in its tree, from its start: an end with another parent too
// had its execution block there as well, and checked its requirements before its start
TEST_F(STNBankTest, each_action_runs_once_in_its_tree)
{
  load("processes-events/p1");
  const auto tree = builder_->get_tree(
    make_plan({{0.0, "(start_charge r1 dock)", 0.0}, {10.0, "(move r1 dock hall)", 0.0}}));
  ASSERT_FALSE(tree.empty());
  for (const std::string action : {"(start_charge r1 dock):0", "(move r1 dock hall):10000"}) {
    size_t count = 0;
    for (auto pos = tree.find("<ExecuteAction action=\"" + action + "\"");
      pos != std::string::npos;
      pos = tree.find("<ExecuteAction action=\"" + action + "\"", pos + 1))
    {
      count++;
    }
    EXPECT_EQ(count, 1u) << action << "\n" << tree;
  }
}

// The action of a plan item, with the duration the plan chose wherever it is read
TEST_F(STNBankTest, plan_items_bind_their_duration)
{
  load("duration-inequalities/p1");
  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  plansys2::DomainExpertClient domain_client(client_node);
  const auto actions = domain_client.getActions();
  plansys2_msgs::msg::PlanItem item;
  item.action = "(charge r1 c1 dock)";
  item.time = 0.0;
  item.duration = 6.5;
  auto action = plansys2::BTBuilder::plan_item_action(domain_client, actions, item);
  ASSERT_TRUE(action.is_durative_action());
  const auto effects = parser::pddl::toString(action.get_at_end_effects());
  EXPECT_EQ(effects.find("?duration"), std::string::npos) << effects;
  EXPECT_NE(effects.find("6.500000"), std::string::npos) << effects;

  // The plan state follows: the charge gives 10 per second of it
  std::vector<plansys2::Predicate> predicates;
  auto battery = parser::pddl::fromStringFunction("(= (battery r1) 0)");
  std::vector<plansys2::Function> functions = {battery};
  ASSERT_TRUE(plansys2::apply(action.get_at_end_effects(), predicates, functions));
  EXPECT_DOUBLE_EQ(functions[0].value, 65.0);

  // Unknown actions give nothing
  item.action = "(fly r1)";
  EXPECT_TRUE(plansys2::BTBuilder::plan_item_action(domain_client, actions, item).is_empty());
}

namespace
{
// PDDL+: the battery only grows with the continuous effect of recharge, and nothing discrete
// orders two recharges in a row
const char * kGreenhouse =
  "(define (domain greenhouse) (:requirements :strips :typing :durative-actions"
  " :numeric-fluents :continuous-effects :duration-inequalities :negative-preconditions)"
  " (:types robot room tank)"
  " (:predicates (robot_at ?r - robot ?l - room) (tank_in ?t - tank ?l - room)"
  "  (valve_open ?t - tank))"
  " (:functions (battery ?r - robot))"
  " (:durative-action move :parameters (?r - robot ?from ?to - room)"
  "  :duration (= ?duration 4)"
  "  :condition (and (at start (robot_at ?r ?from)) (at start (>= (battery ?r) 20)))"
  "  :effect (and (at start (not (robot_at ?r ?from))) (at end (robot_at ?r ?to))"
  "   (decrease (battery ?r) (* #t 2))))"
  " (:durative-action recharge :parameters (?r - robot ?l - room)"
  "  :duration (and (>= ?duration 1) (<= ?duration 10))"
  "  :condition (and (over all (robot_at ?r ?l)))"
  "  :effect (and (increase (battery ?r) (* #t 5))))"
  " (:action open_valve :parameters (?r - robot ?t - tank ?l - room)"
  "  :precondition (and (robot_at ?r ?l) (tank_in ?t ?l) (not (valve_open ?t)))"
  "  :effect (and (valve_open ?t)))"
  " (:action close_valve :parameters (?r - robot ?t - tank ?l - room)"
  "  :precondition (and (robot_at ?r ?l) (tank_in ?t ?l) (valve_open ?t))"
  "  :effect (and (not (valve_open ?t)))))";

const char * kWatering =
  "(define (problem watering) (:domain greenhouse)"
  " (:objects rover - robot dock nursery - room cistern - tank)"
  " (:init (robot_at rover dock) (tank_in cistern nursery) (= (battery rover) 10))"
  " (:goal (and (robot_at rover dock) (not (valve_open cistern)))))";

// As Kairos plans it: happenings that interfere at the same time, in the order of the plan
const std::vector<std::tuple<double, std::string, double>> kWateringPlan = {
  {0.0, "(recharge rover dock)", 1.9}, {1.9, "(recharge rover dock)", 1.0},
  {2.9, "(recharge rover dock)", 1.0}, {3.9, "(move rover dock nursery)", 4.0},
  {7.9, "(open_valve rover cistern nursery)", 0.0},
  {10.9, "(close_valve rover cistern nursery)", 0.0},
  {10.9, "(move rover nursery dock)", 4.0}};
}  // namespace

// Two rates on the same function would add up: a recharge starts once the previous one ends
TEST_F(STNBankTest, continuous_effects_on_a_function_keep_the_plan_order)
{
  load_text(kGreenhouse, kWatering);
  auto graph = builder_->build_stn(make_plan(kWateringPlan));
  EXPECT_TRUE(
    after(
      graph, {"(recharge rover dock)", 0.0, ActionType::END},
      {"(recharge rover dock)", 1.9, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(recharge rover dock)", 1.9, ActionType::END},
      {"(recharge rover dock)", 2.9, ActionType::START}));
}

// What the dynamics give cannot be simulated: the move is still ordered by what it deletes
TEST_F(STNBankTest, a_condition_on_a_dynamic_function_keeps_the_other_orders)
{
  load_text(kGreenhouse, kWatering);
  auto graph = builder_->build_stn(make_plan(kWateringPlan));
  EXPECT_TRUE(
    after(
      graph, {"(recharge rover dock)", 2.9, ActionType::END},
      {"(move rover dock nursery)", 3.9, ActionType::START}));
  EXPECT_TRUE(
    after(
      graph, {"(move rover dock nursery)", 3.9, ActionType::END},
      {"(open_valve rover cistern nursery)", 7.9, ActionType::START}));
}

// At the same time, the plan order: the valve is closed, all of it, before the robot leaves
TEST_F(STNBankTest, interfering_happenings_at_the_same_time_in_the_plan_order)
{
  load_text(kGreenhouse, kWatering);
  auto graph = builder_->build_stn(make_plan(kWateringPlan));
  EXPECT_TRUE(
    after(
      graph, {"(close_valve rover cistern nursery)", 10.9, ActionType::END},
      {"(move rover nursery dock)", 10.9, ActionType::START}));
  EXPECT_FALSE(
    after(
      graph, {"(move rover nursery dock)", 10.9, ActionType::START},
      {"(close_valve rover cistern nursery)", 10.9, ActionType::START}));
  EXPECT_TRUE(builder_->is_executable(make_plan(kWateringPlan)));
}

// A plan of many robots at once: 50 actions, with 40 snaps at one happening. Both builders make
// its tree quickly, keeping the order of each robot
TEST_F(STNBankTest, a_plan_of_many_robots_is_quick)
{
  constexpr int kRobots = 10;
  std::string objects, init;
  for (int i = 0; i < kRobots; i++) {
    const auto n = std::to_string(i);
    objects += " r" + n + " - robot hall" + n + " lab" + n + " - room";
    init += " (robot_at r" + n + " hall" + n + ")";
    for (int j = 0; j < 3; j++) {
      objects += " l" + n + "_" + std::to_string(j) + " - lamp";
      init += " (lamp_in l" + n + "_" + std::to_string(j) + " lab" + n + ")";
    }
  }
  load_text(
    "(define (domain lights) (:requirements :strips :typing :durative-actions"
    " :derived-predicates :existential-preconditions :negative-preconditions)"
    " (:types robot room lamp)"
    " (:predicates (robot_at ?r - robot ?l - room) (lamp_in ?p - lamp ?l - room)"
    "  (lit ?p - lamp) (bright ?l - room))"
    " (:derived (bright ?l - room) (exists (?p - lamp) (and (lamp_in ?p ?l) (lit ?p))))"
    " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 1)"
    "  :condition (and (at start (robot_at ?r ?a)))"
    "  :effect (and (at start (not (robot_at ?r ?a))) (at end (robot_at ?r ?b))))"
    " (:action switch_on :parameters (?r - robot ?p - lamp ?l - room)"
    "  :precondition (and (robot_at ?r ?l) (lamp_in ?p ?l) (not (lit ?p)))"
    "  :effect (and (lit ?p))))",
    "(define (problem p) (:domain lights) (:objects" + objects + ") (:init" + init + ")"
    " (:goal (and)))");
  std::vector<std::tuple<double, std::string, double>> items;
  for (int i = 0; i < kRobots; i++) {
    const auto n = std::to_string(i);
    items.push_back({0.0, "(move r" + n + " hall" + n + " lab" + n + ")", 1.0});
  }
  for (int i = 0; i < kRobots; i++) {
    const auto n = std::to_string(i);
    for (int j = 0; j < 3; j++) {
      items.push_back(
        {1.0, "(switch_on r" + n + " l" + n + "_" + std::to_string(j) + " lab" + n + ")", 0.0});
    }
    items.push_back({1.0, "(move r" + n + " lab" + n + " hall" + n + ")", 1.0});
  }
  const auto plan = make_plan(items);

  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(builder_->is_executable(plan));
  auto graph = builder_->build_stn(plan);
  EXPECT_FALSE(builder_->get_tree(plan).empty());
  const double stn = std::chrono::duration<double>(std::chrono::steady_clock::now() -
    start).count();
  EXPECT_LT(stn, 3.0 * PLANSYS2_TEST_SLOWDOWN);

  // Each robot: in the lab before its lamps, which are on before it leaves
  for (int i = 0; i < kRobots; i++) {
    const auto n = std::to_string(i);
    for (int j = 0; j < 3; j++) {
      const auto lamp = "(switch_on r" + n + " l" + n + "_" + std::to_string(j) + " lab" + n + ")";
      EXPECT_TRUE(
        after(
          graph, {"(move r" + n + " hall" + n + " lab" + n + ")", 0.0, ActionType::END},
          {lamp, 1.0, ActionType::START})) << lamp;
      EXPECT_TRUE(
        after(
          graph, {lamp, 1.0, ActionType::END},
          {"(move r" + n + " lab" + n + " hall" + n + ")", 1.0, ActionType::START})) << lamp;
    }
  }

  plansys2::SimpleBTBuilder simple;
  simple.set_clients(
    std::make_shared<plansys2::DomainExpertClient>(client_node_),
    std::make_shared<plansys2::ProblemExpertClient>(client_node_));
  simple.initialize("", "", 3);
  start = std::chrono::steady_clock::now();
  EXPECT_FALSE(simple.get_tree(plan).empty());
  const double simple_took =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_LT(simple_took, 3.0 * PLANSYS2_TEST_SLOWDOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
// Many snaps at one happening: each one applies after those before it in the plan, without
// trying every subset of them (2^13 here)
TEST_F(STNBankTest, many_actions_at_once_are_quick)
{
  load_text(
    "(define (domain lights) (:requirements :strips :typing :durative-actions"
    " :derived-predicates :existential-preconditions) (:types robot room lamp)"
    " (:predicates (robot_at ?r - robot ?l - room) (lamp_in ?p - lamp ?l - room)"
    "  (lit ?p - lamp) (bright ?l - room))"
    " (:derived (bright ?l - room) (exists (?p - lamp) (and (lamp_in ?p ?l) (lit ?p))))"
    " (:durative-action move :parameters (?r - robot ?a ?b - room) :duration (= ?duration 1)"
    "  :condition (and (at start (robot_at ?r ?a)))"
    "  :effect (and (at start (not (robot_at ?r ?a))) (at end (robot_at ?r ?b))))"
    " (:action switch_on :parameters (?r - robot ?p - lamp ?l - room)"
    "  :precondition (and (robot_at ?r ?l) (lamp_in ?p ?l) (not (lit ?p)))"
    "  :effect (and (lit ?p))))",
    "(define (problem p) (:domain lights) (:objects r1 - robot hall lab - room"
    "  l1 l2 l3 l4 l5 l6 - lamp)"
    " (:init (robot_at r1 hall) (lamp_in l1 lab) (lamp_in l2 lab) (lamp_in l3 lab)"
    "  (lamp_in l4 lab) (lamp_in l5 lab) (lamp_in l6 lab))"
    " (:goal (and (bright lab))))");
  std::vector<std::tuple<double, std::string, double>> items = {{0.0, "(move r1 hall lab)", 1.0}};
  for (int i = 1; i <= 6; i++) {
    items.push_back({1.0, "(switch_on r1 l" + std::to_string(i) + " lab)", 0.0});
  }
  const auto start = std::chrono::steady_clock::now();
  auto graph = builder_->build_stn(make_plan(items));
  EXPECT_TRUE(builder_->is_executable(make_plan(items)));
  const double took =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  // Every subset of them took seconds
  EXPECT_LT(took, 1.0);
  for (int i = 1; i <= 6; i++) {
    EXPECT_TRUE(
      after(
        graph, {"(move r1 hall lab)", 0.0, ActionType::END},
        {"(switch_on r1 l" + std::to_string(i) + " lab)", 1.0, ActionType::START})) << i;
  }
}
