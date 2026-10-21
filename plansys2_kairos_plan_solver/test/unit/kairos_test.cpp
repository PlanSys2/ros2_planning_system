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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "kairos/kairos.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_kairos_plan_solver/kairos_plan_solver.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "pluginlib/class_loader.hpp"

using namespace std::chrono_literals;

namespace
{

std::string read_pddl(const std::string & name)
{
  const auto path =
    plansys2::share_path("plansys2_kairos_plan_solver") / "pddl" / name;
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// Sequential and numeric: fuel runs out after two moves, so the robot must refuel
const char * kSeqDomain =
  R"(
(define (domain seq)
  (:requirements :strips :typing :numeric-fluents)
  (:types room)
  (:predicates (at ?r - room) (connected ?a ?b - room))
  (:functions (fuel))
  (:action move
    :parameters (?a ?b - room)
    :precondition (and (at ?a) (connected ?a ?b) (>= (fuel) 1))
    :effect (and (not (at ?a)) (at ?b) (decrease (fuel) 1)))
  (:action refuel
    :parameters (?a - room)
    :precondition (and (at ?a) (< (fuel) 1))
    :effect (assign (fuel) 2)))
)";

const char * kSeqProblem =
  R"(
(define (problem seq1)
  (:domain seq)
  (:objects r1 r2 r3 r4 - room)
  (:init (at r1) (connected r1 r2) (connected r2 r3) (connected r3 r4) (= (fuel) 2))
  (:goal (at r4)))
)";

struct Fixture
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<plansys2::KairosPlanSolver> solver;
};

Fixture make_solver(const std::string & name = "test_node")
{
  Fixture f;
  f.node = rclcpp_lifecycle::LifecycleNode::make_shared(name);
  f.solver = std::make_shared<plansys2::KairosPlanSolver>();
  f.solver->configure(f.node, "KAIROS");
  return f;
}

// The plan message validated by Kairos against the problem
void expect_valid(
  const std::string & domain, const std::string & problem, const plansys2_msgs::msg::Plan & msg,
  bool temporal)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(6);
  for (const auto & item : msg.items) {
    if (temporal) {
      text << item.time << ": " << item.action << " [" << item.duration << "]\n";
    } else {
      text << item.action << "\n";
    }
  }
  auto d = kairos::parse_domain(domain);
  ASSERT_TRUE(d);
  auto p = kairos::parse_problem(problem, *d);
  ASSERT_TRUE(p);
  auto plan = kairos::parse_plan(text.str());
  ASSERT_TRUE(plan) << text.str();
  kairos::ValidateOptions options;
  options.epsilon = 0.001;
  const auto report = kairos::validate(*p, *plan, options);
  EXPECT_TRUE(report.valid()) << text.str();
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

TEST(kairos_plan_solver, generate_plan_good)
{
  auto f = make_solver();
  const auto domain = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");

  auto plan = f.solver->getPlan(domain, problem, "generate_plan_good");

  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 3u);
  EXPECT_EQ(plan->items[0].action, "(move leia kitchen bedroom)");
  EXPECT_EQ(plan->items[1].action, "(approach leia bedroom jack)");
  EXPECT_EQ(plan->items[2].action, "(talk leia jack jack m1)");
  for (size_t i = 0; i < plan->items.size(); i++) {
    EXPECT_FLOAT_EQ(plan->items[i].duration, 5.0f);
    if (i > 0) {
      EXPECT_GT(plan->items[i].time, plan->items[i - 1].time);
    }
  }
  EXPECT_FLOAT_EQ(plan->items[0].time, 0.0f);
  expect_valid(domain, problem, *plan, true);
}

TEST(kairos_plan_solver, load_plugin_and_plan)
{
  pluginlib::ClassLoader<plansys2::PlanSolverBase> loader(
    "plansys2_core", "plansys2::PlanSolverBase");
  plansys2::PlanSolverBase::Ptr plugin;
  ASSERT_NO_THROW(plugin = loader.createUniqueInstance("plansys2/KairosPlanSolver"));
  ASSERT_TRUE(plugin);

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("plugin_node");
  plugin->configure(node, "KAIROS");
  EXPECT_TRUE(node->has_parameter("KAIROS.anytime"));
  EXPECT_TRUE(node->has_parameter("KAIROS.no_improvement"));
  EXPECT_DOUBLE_EQ(node->get_parameter("KAIROS.no_improvement").as_double(), 0.1);
  EXPECT_TRUE(node->has_parameter("KAIROS.search"));

  auto plan = plugin->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->items.size(), 3u);
  plugin.reset();
}

TEST(kairos_plan_solver, configure_keeps_declared_parameters)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("declared_node");
  node->declare_parameter("KAIROS.no_improvement", 0.3);
  node->declare_parameter("KAIROS.search", std::string("lama"));
  auto solver = std::make_shared<plansys2::KairosPlanSolver>();
  ASSERT_NO_THROW(solver->configure(node, "KAIROS"));
  EXPECT_DOUBLE_EQ(node->get_parameter("KAIROS.no_improvement").as_double(), 0.3);
  EXPECT_EQ(node->get_parameter("KAIROS.search").as_string(), "lama");

  // Configured twice, as after a cleanup
  ASSERT_NO_THROW(solver->configure(node, "KAIROS"));
  EXPECT_TRUE(solver->getPlan(kSeqDomain, kSeqProblem));
}

TEST(kairos_plan_solver, check_domains)
{
  auto f = make_solver();
  EXPECT_TRUE(f.solver->isDomainValid(read_pddl("domain_1_ok.pddl"), "check_1_ok_domain"));
  EXPECT_FALSE(f.solver->isDomainValid(read_pddl("domain_2_error.pddl"), "check_2_error"));
  EXPECT_TRUE(f.solver->isDomainValid(read_pddl("domain_simple.pddl")));
  EXPECT_TRUE(f.solver->isDomainValid(read_pddl("suave_domain.pddl")));
  EXPECT_TRUE(f.solver->isDomainValid(kSeqDomain));
  EXPECT_FALSE(f.solver->isDomainValid(""));
  EXPECT_FALSE(f.solver->isDomainValid("(define (domain broken)"));
}

TEST(kairos_plan_solver, unsolvable_is_fast)
{
  auto f = make_solver();
  const auto start = std::chrono::steady_clock::now();
  auto plan = f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_2.pddl"), "", 10s);
  EXPECT_FALSE(plan);
  // Proved, not timed out
  EXPECT_LT(seconds_since(start), 1.0);
}

TEST(kairos_plan_solver, many_durative_actions_unsolvable)
{
  // 12800 durative actions: POPF cannot discard it, Kairos proves it without a timeout
  auto f = make_solver();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_hard_unsolvable.pddl"), "", 10s));
  EXPECT_LT(seconds_since(start), 2.0);
}

TEST(kairos_plan_solver, invalid_problem_and_domain)
{
  auto f = make_solver();
  const auto domain = read_pddl("domain_simple.pddl");
  EXPECT_FALSE(f.solver->getPlan(domain, read_pddl("problem_simple_3.pddl")));
  EXPECT_FALSE(f.solver->getPlan(domain, ""));
  EXPECT_FALSE(f.solver->getPlan(read_pddl("domain_2_error.pddl"),
    read_pddl("problem_simple_1.pddl")));
  EXPECT_FALSE(f.solver->getPlan("", read_pddl("problem_simple_1.pddl")));
  // A problem of another domain
  EXPECT_FALSE(f.solver->getPlan(domain, kSeqProblem));

  // Errors leave nothing behind
  auto plan = f.solver->getPlan(domain, read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->items.size(), 3u);
}

TEST(kairos_plan_solver, domains_alternate)
{
  // The cached domain must follow the domain of each call
  auto f = make_solver();
  const auto simple = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  for (int i = 0; i < 3; i++) {
    auto temporal = f.solver->getPlan(simple, problem);
    ASSERT_TRUE(temporal);
    EXPECT_EQ(temporal->items.size(), 3u);
    auto sequential = f.solver->getPlan(kSeqDomain, kSeqProblem);
    ASSERT_TRUE(sequential);
    expect_valid(kSeqDomain, kSeqProblem, *sequential, false);
  }

  // Same name, different content
  std::string changed = simple;
  const auto pos = changed.find("( = ?duration 5)");
  ASSERT_NE(pos, std::string::npos);
  changed.replace(pos, 16, "( = ?duration 7)");
  auto plan = f.solver->getPlan(changed, problem);
  ASSERT_TRUE(plan);
  EXPECT_FLOAT_EQ(plan->items[0].duration, 7.0f);
}

TEST(kairos_plan_solver, more_domains_than_the_cache)
{
  // Six versions of the domain, by duration, cycled twice: evicted ones are parsed again
  auto f = make_solver();
  const auto simple = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  for (int round = 0; round < 2; round++) {
    for (int d = 1; d <= 6; d++) {
      std::string domain = simple;
      size_t pos = 0;
      while ((pos = domain.find("( = ?duration 5)", pos)) != std::string::npos) {
        const auto duration = "( = ?duration " + std::to_string(d) + ")";
        domain.replace(pos, 16, duration);
        pos += duration.size();
      }
      auto plan = f.solver->getPlan(domain, problem);
      ASSERT_TRUE(plan) << d;
      ASSERT_EQ(plan->items.size(), 3u);
      for (const auto & item : plan->items) {
        EXPECT_FLOAT_EQ(item.duration, static_cast<float>(d));
      }
    }
  }
}

// The plan message as Kairos reads it: temporal if the domain has durative actions
kairos::Plan to_kairos(const plansys2_msgs::msg::Plan & msg, const std::string & domain)
{
  kairos::Plan plan;
  plan.temporal = domain.find(":durative-action") != std::string::npos || msg.has_end;
  for (size_t i = 0; i < msg.items.size(); i++) {
    auto text = msg.items[i].action.substr(1, msg.items[i].action.size() - 2);
    std::istringstream words(text);
    kairos::PlanStep step;
    words >> step.name;
    for (std::string arg; words >> arg; ) {
      step.args.push_back(arg);
    }
    step.time = plan.temporal ? msg.items[i].time : static_cast<double>(i);
    step.duration = plan.temporal ? msg.items[i].duration : 0.0;
    plan.steps.push_back(step);
  }
  if (msg.has_end) {
    plan.end = msg.end;
  }
  return plan;
}

// Every task of the bank, each feature with solvable and unsolvable problems
TEST(kairos_plan_solver, the_bank)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  ASSERT_GE(cases.size(), 60u);
  auto f = make_solver("kairos_bank");
  f.node->set_parameter(rclcpp::Parameter("KAIROS.anytime", false));  // The first plan is enough
  const auto supported = f.solver->supported_requirements();

  const auto all = std::chrono::steady_clock::now();
  for (const auto & task : cases) {
    SCOPED_TRACE(task.name);
    EXPECT_TRUE(plansys2::missing_requirements(task.uses, supported).empty());
    const auto start = std::chrono::steady_clock::now();
    const auto plan = f.solver->getPlan(task.domain, task.problem, "", 5s);
    EXPECT_LT(seconds_since(start), 2.0);
    ASSERT_EQ(plan.has_value(), task.solvable);
    if (plan) {
      kairos::ValidateOptions options;
      options.epsilon = 0.001;
      const auto report =
        kairos::validate_plan(task.domain, task.problem, to_kairos(*plan, task.domain), options);
      EXPECT_TRUE(report.valid()) << (report.diagnostics.empty() ? "" :
      kairos::to_string(report.diagnostics[0]));
      EXPECT_TRUE(plan->has_cost);
    }
  }
  EXPECT_LT(seconds_since(all), 15.0);
}

TEST(kairos_plan_solver, plans_with_every_requirement)
{
  auto f = make_solver();
  EXPECT_EQ(f.solver->supported_requirements(), plansys2::known_pddl_requirements());
}

TEST(kairos_plan_solver, sequential_numeric_plan)
{
  auto f = make_solver();
  auto plan = f.solver->getPlan(kSeqDomain, kSeqProblem);
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 4u);  // move, move, refuel, move
  for (size_t i = 0; i < plan->items.size(); i++) {
    EXPECT_FLOAT_EQ(plan->items[i].time, static_cast<float>(i));
    EXPECT_FLOAT_EQ(plan->items[i].duration, 0.0f);
  }
  EXPECT_EQ(plan->items[2].action, "(refuel r3)");
  expect_valid(kSeqDomain, kSeqProblem, *plan, false);

  // Without a metric, the cost of a sequential plan is its length; only PDDL+ plans have an end
  EXPECT_TRUE(plan->has_cost);
  EXPECT_DOUBLE_EQ(plan->cost, 4.0);
  EXPECT_FALSE(plan->has_end);
}

// Each move takes a day and a few ms: in float32 the later times lose the ms
TEST(kairos_plan_solver, long_plans_keep_their_decimals)
{
  const char * domain =
    R"(
(define (domain days)
  (:requirements :strips :typing :durative-actions)
  (:types room)
  (:predicates (at ?r - room) (connected ?a ?b - room))
  (:durative-action move
    :parameters (?a ?b - room)
    :duration (= ?duration 86400.003)
    :condition (and (at start (at ?a)) (over all (connected ?a ?b)))
    :effect (and (at start (not (at ?a))) (at end (at ?b)))))
)";
  const char * problem =
    R"(
(define (problem days1)
  (:domain days)
  (:objects r1 r2 r3 r4 - room)
  (:init (at r1) (connected r1 r2) (connected r2 r3) (connected r3 r4))
  (:goal (at r4)))
)";
  auto f = make_solver();
  auto plan = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 3u);
  for (size_t i = 0; i < plan->items.size(); i++) {
    EXPECT_NEAR(plan->items[i].duration, 86400.003, 1e-6);
    if (i > 0) {
      // The next one starts after the previous one ends, by less than a ms
      const auto gap = plan->items[i].time -
        (plan->items[i - 1].time + plan->items[i - 1].duration);
      EXPECT_GT(gap, 0.0);
      EXPECT_LT(gap, 0.01);
    }
  }
  // The makespan, three days and some ms
  EXPECT_TRUE(plan->has_cost);
  EXPECT_NEAR(plan->cost, plan->items[2].time + plan->items[2].duration, 1e-6);
  expect_valid(domain, problem, *plan, true);
}

TEST(kairos_plan_solver, derived_predicates)
{
  auto f = make_solver();
  const auto domain = read_pddl("suave_domain.pddl");
  const auto problem = read_pddl("suave_problem.pddl");
  auto plan = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(plan);
  EXPECT_FALSE(plan->items.empty());
  expect_valid(domain, problem, *plan, false);
}

TEST(kairos_plan_solver, temporal_elevators)
{
  auto f = make_solver();
  const auto domain = read_pddl("elevator_domain.pddl");
  const auto problem = read_pddl("elevator_problem.pddl");
  auto plan = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(plan);
  EXPECT_FALSE(plan->items.empty());
  expect_valid(domain, problem, *plan, true);
}

TEST(kairos_plan_solver, node_timeout_is_respected)
{
  auto f = make_solver();
  const auto start = std::chrono::steady_clock::now();
  auto plan = f.solver->getPlan(
    read_pddl("puzzle_domain.pddl"), read_pddl("puzzle_unsolvable.pddl"), "", 1s);
  const double elapsed = seconds_since(start);
  EXPECT_FALSE(plan);
  // Returns before the planner node deadline
  EXPECT_LT(elapsed, 1.0);
  EXPECT_GT(elapsed, 0.5);
}

TEST(kairos_plan_solver, short_node_timeouts)
{
  auto f = make_solver();
  const auto domain = read_pddl("puzzle_domain.pddl");
  const auto hard = read_pddl("puzzle_unsolvable.pddl");
  for (auto timeout : {rclcpp::Duration(0ms), rclcpp::Duration(1ms), rclcpp::Duration(30ms)}) {
    const auto start = std::chrono::steady_clock::now();
    EXPECT_FALSE(f.solver->getPlan(domain, hard, "", timeout));
    EXPECT_LT(seconds_since(start), 0.5);
  }
}

TEST(kairos_plan_solver, timeout_parameter)
{
  auto f = make_solver();
  f.node->set_parameter(rclcpp::Parameter("KAIROS.timeout", 0.3));
  const auto start = std::chrono::steady_clock::now();
  auto plan = f.solver->getPlan(
    read_pddl("puzzle_domain.pddl"), read_pddl("puzzle_unsolvable.pddl"), "", 20s);
  EXPECT_FALSE(plan);
  EXPECT_LT(seconds_since(start), 0.6);

  // Larger than the node timeout: the node one wins
  f.node->set_parameter(rclcpp::Parameter("KAIROS.timeout", 60.0));
  const auto start2 = std::chrono::steady_clock::now();
  EXPECT_FALSE(f.solver->getPlan(
    read_pddl("puzzle_domain.pddl"), read_pddl("puzzle_unsolvable.pddl"), "", 500ms));
  EXPECT_LT(seconds_since(start2), 0.5);
}

TEST(kairos_plan_solver, cancel_stops_the_search)
{
  auto f = make_solver();
  const auto domain = read_pddl("puzzle_domain.pddl");
  const auto hard = read_pddl("puzzle_unsolvable.pddl");
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(
    std::launch::async, [&]() {return f.solver->getPlan(domain, hard, "", 30s);});
  // Kept: cancel once the search is running, which is not observable
  std::this_thread::sleep_for(200ms);
  f.solver->cancel();
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get());
  EXPECT_LT(seconds_since(start), 0.5);

  // A cancel from before does not stop the next call
  f.solver->cancel();
  auto plan = f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->items.size(), 3u);
}

TEST(kairos_plan_solver, cancel_keeps_the_best_plan)
{
  // Anytime without a no-improvement limit: cancel returns the plan found so far
  auto f = make_solver();
  f.node->set_parameter(rclcpp::Parameter("KAIROS.no_improvement", 0.0));
  const auto domain = read_pddl("elevator_domain.pddl");
  const auto problem = read_pddl("elevator_problem.pddl");
  auto future = std::async(
    std::launch::async, [&]() {return f.solver->getPlan(domain, problem, "", 30s);});
  // Kept: time for the search to find a first plan, which is not observable
  std::this_thread::sleep_for(100ms);
  f.solver->cancel();
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  auto plan = future.get();
  ASSERT_TRUE(plan);
  expect_valid(domain, problem, *plan, true);
}

TEST(kairos_plan_solver, anytime_options)
{
  auto f = make_solver();
  const auto domain = read_pddl("elevator_domain.pddl");
  const auto problem = read_pddl("elevator_problem.pddl");

  f.node->set_parameter(rclcpp::Parameter("KAIROS.anytime", false));
  auto first = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(first);
  expect_valid(domain, problem, *first, true);

  f.node->set_parameter(rclcpp::Parameter("KAIROS.anytime", true));
  f.node->set_parameter(rclcpp::Parameter("KAIROS.no_improvement", 0.05));
  auto improved = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(improved);
  expect_valid(domain, problem, *improved, true);
}

TEST(kairos_plan_solver, search_and_engine_parameters)
{
  auto f = make_solver();
  const auto domain = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");

  for (const auto & preset : kairos::search_presets()) {
    f.node->set_parameter(rclcpp::Parameter("KAIROS.search", preset));
    auto plan = f.solver->getPlan(kSeqDomain, kSeqProblem);
    ASSERT_TRUE(plan) << preset;
    expect_valid(kSeqDomain, kSeqProblem, *plan, false);
  }
  f.node->set_parameter(rclcpp::Parameter("KAIROS.search", "auto"));

  for (const std::string engine : {"m2", "m3", "auto"}) {
    f.node->set_parameter(rclcpp::Parameter("KAIROS.engine", engine));
    auto plan = f.solver->getPlan(domain, problem);
    ASSERT_TRUE(plan) << engine;
    expect_valid(domain, problem, *plan, true);
  }
}

TEST(kairos_plan_solver, unknown_parameter_values)
{
  // Warned and replaced by the defaults
  auto f = make_solver();
  f.node->set_parameter(rclcpp::Parameter("KAIROS.search", "nonsense"));
  f.node->set_parameter(rclcpp::Parameter("KAIROS.engine", "m9"));
  f.node->set_parameter(rclcpp::Parameter("KAIROS.log_level", "loud"));
  f.node->set_parameter(rclcpp::Parameter("KAIROS.memory_limit_mb", -5));
  f.node->set_parameter(rclcpp::Parameter("KAIROS.no_improvement", -1.0));
  f.node->set_parameter(rclcpp::Parameter("KAIROS.timeout", -3.0));
  auto plan = f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->items.size(), 3u);
}

TEST(kairos_plan_solver, threads)
{
  auto f = make_solver();
  EXPECT_EQ(f.node->get_parameter("KAIROS.threads").as_int(), 1);
  const auto domain = read_pddl("elevator_domain.pddl");
  const auto problem = read_pddl("elevator_problem.pddl");
  for (int threads : {0, 1, 3}) {
    f.node->set_parameter(rclcpp::Parameter("KAIROS.threads", threads));
    auto plan = f.solver->getPlan(domain, problem);
    ASSERT_TRUE(plan) << threads;
    expect_valid(domain, problem, *plan, true);
  }
}

TEST(kairos_plan_solver, log_levels)
{
  auto f = make_solver();
  for (const std::string level : {"debug", "info", "warn", "warning", "error"}) {
    f.node->set_parameter(rclcpp::Parameter("KAIROS.log_level", level));
    EXPECT_TRUE(f.solver->getPlan(kSeqDomain, kSeqProblem)) << level;
  }
}

TEST(kairos_plan_solver, dump_dir)
{
  const auto dir = std::filesystem::temp_directory_path() /
    ("kairos_plugin_dump_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  auto f = make_solver();
  f.node->set_parameter(rclcpp::Parameter("KAIROS.dump_dir", dir.string()));
  EXPECT_TRUE(f.solver->getPlan(kSeqDomain, kSeqProblem));
  ASSERT_TRUE(std::filesystem::exists(dir));
  EXPECT_FALSE(std::filesystem::is_empty(dir));
  std::filesystem::remove_all(dir);
}

TEST(kairos_plan_solver, parallel_calls)
{
  // The planner node may run several requests on the same solver at once
  auto f = make_solver();
  const auto simple = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  std::vector<std::future<std::optional<plansys2_msgs::msg::Plan>>> futures;
  for (int i = 0; i < 8; i++) {
    futures.push_back(
      std::async(
        std::launch::async, [&, i]() {
          return i % 2 ? f.solver->getPlan(simple, problem) :
                 f.solver->getPlan(kSeqDomain, kSeqProblem);
        }));
  }
  for (int i = 0; i < 8; i++) {
    auto plan = futures[i].get();
    ASSERT_TRUE(plan) << i;
    EXPECT_EQ(plan->items.size(), i % 2 ? 3u : 4u);
  }
}

TEST(kairos_plan_solver, replanning_is_fast)
{
  // PlanSys2 replans with the same domain: after the first call it must take milliseconds
  auto f = make_solver();
  const auto domain = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  ASSERT_TRUE(f.solver->getPlan(domain, problem));
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 20; i++) {
    ASSERT_TRUE(f.solver->getPlan(domain, problem));
  }
  EXPECT_LT(seconds_since(start) / 20, 0.1);
}

namespace
{

// A kairos executable that misbehaves: a script with the given body, in a folder of its own
std::string fake_kairos(const std::string & name, const std::string & body)
{
  const auto dir = std::filesystem::temp_directory_path() / ("plansys2_fake_kairos_" + name);
  std::filesystem::create_directories(dir);
  const auto path = dir / "kairos";
  std::ofstream(path) << "#!/bin/sh\n" << body << "\n";
  std::filesystem::permissions(path, std::filesystem::perms::owner_all);
  return path.string();
}

Fixture make_isolated(const std::string & name)
{
  auto f = make_solver(name);
  f.node->set_parameter(rclcpp::Parameter("KAIROS.isolated", true));
  return f;
}

}  // namespace

TEST(kairos_plan_solver, isolated_plans_as_in_process)
{
  auto f = make_isolated("isolated_plans");
  const auto domain = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  auto plan = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 3u);
  EXPECT_EQ(plan->items[0].action, "(move leia kitchen bedroom)");
  EXPECT_FLOAT_EQ(plan->items[0].duration, 5.0f);
  expect_valid(domain, problem, *plan, true);

  auto seq = f.solver->getPlan(kSeqDomain, kSeqProblem);
  ASSERT_TRUE(seq);
  EXPECT_EQ(seq->items.size(), 4u);
  expect_valid(kSeqDomain, kSeqProblem, *seq, false);

  const auto elevators = read_pddl("elevator_domain.pddl");
  const auto elevators_problem = read_pddl("elevator_problem.pddl");
  auto temporal = f.solver->getPlan(elevators, elevators_problem);
  ASSERT_TRUE(temporal);
  expect_valid(elevators, elevators_problem, *temporal, true);
}

TEST(kairos_plan_solver, isolated_without_plan)
{
  auto f = make_isolated("isolated_without_plan");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_2.pddl"), "", 10s));
  EXPECT_LT(seconds_since(start), 2.0);  // Proved unsolvable, not timed out
  EXPECT_FALSE(f.solver->getPlan(read_pddl("domain_simple.pddl"), "(define (problem"));
  EXPECT_FALSE(f.solver->getPlan("(define (domain", read_pddl("problem_simple_1.pddl")));
}

TEST(kairos_plan_solver, isolated_respects_the_node_timeout)
{
  auto f = make_isolated("isolated_timeout");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(f.solver->getPlan(
    read_pddl("puzzle_domain.pddl"), read_pddl("puzzle_unsolvable.pddl"), "", 1s));
  EXPECT_LT(seconds_since(start), 1.5);
}

TEST(kairos_plan_solver, isolated_cancel)
{
  auto f = make_isolated("isolated_cancel");
  const auto domain = read_pddl("puzzle_domain.pddl");
  const auto hard = read_pddl("puzzle_unsolvable.pddl");
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(
    std::launch::async, [&]() {return f.solver->getPlan(domain, hard, "", 30s);});
  std::this_thread::sleep_for(300ms);
  f.solver->cancel();
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  EXPECT_FALSE(future.get());
  EXPECT_LT(seconds_since(start), 1.5);
  ASSERT_TRUE(
    f.solver->getPlan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl")));
}

TEST(kairos_plan_solver, isolated_survives_a_crashing_kairos)
{
  // What isolated is for: a Kairos that crashes ends this plan, not the planner node
  auto f = make_isolated("isolated_crash");
  const auto domain = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  for (const auto & body : {std::string("kill -SEGV $$"), std::string("kill -ABRT $$"),
      std::string("exit 3"), std::string("echo 'not a plan ((('")})
  {
    SCOPED_TRACE(body);
    f.node->set_parameter(
      rclcpp::Parameter("KAIROS.executable", fake_kairos("crash", body)));
    EXPECT_FALSE(f.solver->getPlan(domain, problem));
  }
  // The real one, again
  f.node->set_parameter(rclcpp::Parameter("KAIROS.executable", std::string("")));
  auto plan = f.solver->getPlan(domain, problem);
  ASSERT_TRUE(plan);
  EXPECT_EQ(plan->items.size(), 3u);
}

TEST(kairos_plan_solver, isolated_survives_a_hanging_kairos)
{
  // A Kairos that ignores its own timeout is killed at the node timeout
  auto f = make_isolated("isolated_hang");
  f.node->set_parameter(
    rclcpp::Parameter("KAIROS.executable", fake_kairos("hang", "exec sleep 60")));
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"), "", 1s));
  EXPECT_LT(seconds_since(start), 3.0);
}

TEST(kairos_plan_solver, isolated_without_executable)
{
  auto f = make_isolated("isolated_missing");
  f.node->set_parameter(
    rclcpp::Parameter("KAIROS.executable", std::string("/nonexistent/kairos")));
  EXPECT_FALSE(f.solver->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl")));
}

TEST(kairos_plan_solver, isolated_parallel_calls)
{
  auto f = make_isolated("isolated_parallel");
  const auto simple = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");
  std::vector<std::future<std::optional<plansys2_msgs::msg::Plan>>> futures;
  for (int i = 0; i < 6; i++) {
    futures.push_back(
      std::async(
        std::launch::async, [&, i]() {
          return i % 2 ? f.solver->getPlan(simple, problem) :
                 f.solver->getPlan(kSeqDomain, kSeqProblem);
        }));
  }
  for (int i = 0; i < 6; i++) {
    auto plan = futures[i].get();
    ASSERT_TRUE(plan) << i;
    EXPECT_EQ(plan->items.size(), i % 2 ? 3u : 4u);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
