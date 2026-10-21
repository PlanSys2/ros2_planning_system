// Copyright 2019 Intelligent Robotics Lab
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

#include <unistd.h>

#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <future>
#include <chrono>

#include "plansys2_core/PackagePaths.hpp"

#include "gtest/gtest.h"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

#include "pluginlib/class_loader.hpp"
#include "kairos/kairos.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_core/TestUtils.hpp"

void test_plan_generation(const std::string & argument = "")
{
  std::string pkgpath =
    plansys2::share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");
  node->set_parameter(rclcpp::Parameter("POPF.arguments", argument));

  auto plan = planner->getPlan(domain_str, problem_str, "generate_plan_good");

  ASSERT_TRUE(plan);
  ASSERT_EQ(plan.value().items.size(), 3);
  ASSERT_EQ(plan.value().items[0].action, "(move leia kitchen bedroom)");
  ASSERT_EQ(plan.value().items[1].action, "(approach leia bedroom jack)");
  ASSERT_EQ(plan.value().items[2].action, "(talk leia jack jack m1)");
}

std::optional<std::filesystem::path> test_folder_creation(
  const std::string & output_dir = "", const std::string & node_namespace = "")
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();

  planner->configure(node, "POPF");
  if (!output_dir.empty()) {
    node->set_parameter(rclcpp::Parameter("POPF.output_dir", output_dir));
  }

  return planner->create_folders(node_namespace);
}

TEST(popf_plan_solver, generate_plan_good)
{
  test_plan_generation();
}

TEST(popf_plan_solver, generate_plan_good_with_argument)
{
  test_plan_generation("-e");
}

TEST(popf_plan_solver, load_popf_plugin)
{
  try {
    pluginlib::ClassLoader<plansys2::PlanSolverBase> lp_loader(
      "plansys2_core", "plansys2::PlanSolverBase");
    plansys2::PlanSolverBase::Ptr plugin =
      lp_loader.createUniqueInstance("plansys2/POPFPlanSolver");
    ASSERT_TRUE(true);
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    ASSERT_TRUE(false);
  }
}

TEST(popf_plan_solver, check_1_ok_domain)
{
  std::string pkgpath =
    plansys2::share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_1_ok.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  bool result = planner->isDomainValid(domain_str, "check_1_ok_domain");

  ASSERT_TRUE(result);
}

TEST(popf_plan_solver, check_2_error_domain)
{
  std::string pkgpath =
    plansys2::share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_2_error.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  bool result = planner->isDomainValid(domain_str, "check_2_error_domain");

  ASSERT_FALSE(result);
}

TEST(popf_plan_solver, generate_plan_unsolvable)
{
  std::string pkgpath =
    plansys2::share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_2.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str);

  ASSERT_FALSE(plan);
}

TEST(popf_plan_solver, generate_plan_error)
{
  std::string pkgpath =
    plansys2::share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_3.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str);

  ASSERT_FALSE(plan);
}

TEST(popf_plan_solver, create_folder_default)
{
  const auto output_dir = test_folder_creation();
  ASSERT_TRUE(output_dir.has_value());
  EXPECT_EQ(output_dir.value(), std::filesystem::temp_directory_path());
}

TEST(popf_plan_solver, create_folder_custom_path)
{
  const auto test_path = std::filesystem::temp_directory_path() / "test" / "path" / "one";
  const auto output_dir = test_folder_creation(test_path);
  ASSERT_TRUE(output_dir.has_value());
  EXPECT_EQ(output_dir.value(), test_path);
}

TEST(popf_plan_solver, create_folder_custom_path_and_namespace)
{
  const auto test_path = std::filesystem::temp_directory_path() / "test" / "path" / "two";
  const auto test_namespace = "/test/node";
  const auto output_dir = test_folder_creation(test_path, test_namespace);
  ASSERT_TRUE(output_dir.has_value());
  EXPECT_EQ(output_dir.value(), test_path / "test" / "node");
}

TEST(popf_plan_solver, create_folder_filesystem_error)
{
  const auto test_path = std::filesystem::temp_directory_path() / "test" / "path" / "three";

  // Create a file at the test path to force a filesystem error
  std::ofstream out(test_path);
  out << "random text\n";
  out.close();

  const auto test_namespace = "/test/node";
  const auto output_dir = test_folder_creation(test_path, test_namespace);
  ASSERT_FALSE(output_dir.has_value());
}

std::string read_test_file(const std::string & name)
{
  std::string pkgpath =
    plansys2::share_path("plansys2_popf_plan_solver").string();
  std::ifstream ifs(pkgpath + "/pddl/" + name);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

// Number of running processes whose command line mentions `pattern`
int count_processes_with(const std::string & pattern)
{
  int count = 0;
  for (const auto & entry : std::filesystem::directory_iterator("/proc")) {
    const auto name = entry.path().filename().string();
    if (name.find_first_not_of("0123456789") != std::string::npos ||
      std::stoi(name) == getpid())
    {
      continue;
    }
    std::ifstream cmdline_ifs(entry.path() / "cmdline");
    std::string cmdline((std::istreambuf_iterator<char>(cmdline_ifs)),
      std::istreambuf_iterator<char>());
    if (cmdline.find(pattern) != std::string::npos) {
      count++;
    }
  }
  return count;
}

// popf, and anything it starts, must not survive the call (#418)
TEST(popf_plan_solver, timeout_is_enforced_and_popf_is_killed)
{
  const std::string ns = "popf_timeout_test_" + std::to_string(getpid());
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto start = std::chrono::steady_clock::now();
  auto plan = planner->getPlan(
    read_test_file("domain_simple.pddl"), read_test_file("problem_hard_unsolvable.pddl"),
    ns, rclcpp::Duration(2s));
  auto elapsed = std::chrono::steady_clock::now() - start;

  ASSERT_FALSE(plan);
  ASSERT_GE(elapsed, 2s);
  ASSERT_LT(elapsed, 5s);

  // popf and its children are gone within 500 ms
  plansys2::testing::wait_for([&]() {return count_processes_with(ns) == 0;}, 500ms);
  ASSERT_EQ(count_processes_with(ns), 0);

  // The solver is still usable after a timeout
  auto good = planner->getPlan(
    read_test_file("domain_simple.pddl"), read_test_file("problem_simple_1.pddl"), ns);
  ASSERT_TRUE(good);
  ASSERT_EQ(good.value().items.size(), 3u);
}

TEST(popf_plan_solver, cancel_stops_popf)
{
  const std::string ns = "popf_cancel_test_" + std::to_string(getpid());
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto future = std::async(
    std::launch::async, [&]() {
      return planner->getPlan(
        read_test_file("domain_simple.pddl"), read_test_file("problem_hard_unsolvable.pddl"),
        ns, rclcpp::Duration(60s));
    });

  // Wait until popf is actually running
  plansys2::testing::wait_for([&]() {return count_processes_with(ns) > 0;});
  ASSERT_GT(count_processes_with(ns), 0);

  planner->cancel();
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  ASSERT_FALSE(future.get());

  plansys2::testing::wait_for([&]() {return count_processes_with(ns) == 0;}, 500ms);
  ASSERT_EQ(count_processes_with(ns), 0);
}

// #441: popf I/O

class TestablePOPFPlanSolver : public plansys2::POPFPlanSolver
{
public:
  using POPFPlanSolver::parse_plan_result;
};

// A fresh folder for a test, removed first if a previous run left it
std::filesystem::path fresh_folder(const std::string & name)
{
  auto path = std::filesystem::temp_directory_path() /
    (name + "_" + std::to_string(getpid()));
  std::filesystem::remove_all(path);
  return path;
}

std::shared_ptr<plansys2::POPFPlanSolver> make_solver(
  const std::string & node_name, const std::filesystem::path & output_dir = {},
  const std::string & arguments = "")
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared(node_name);
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");
  if (!output_dir.empty()) {
    node->set_parameter(rclcpp::Parameter("POPF.output_dir", output_dir.string()));
  }
  node->set_parameter(rclcpp::Parameter("POPF.arguments", arguments));
  return planner;
}

// The robot starts at `room`: from kitchen the plan has 3 actions, from bedroom 2
std::string problem_from(const std::string & room)
{
  auto problem = read_test_file("problem_simple_1.pddl");
  const std::string init = "(robot_at leia kitchen)";
  problem.replace(problem.find(init), init.size(), "(robot_at leia " + room + ")");
  return problem;
}

TEST(popf_plan_solver, arguments_reach_popf)
{
  const auto domain = read_test_file("domain_simple.pddl");
  const auto problem = read_test_file("problem_simple_1.pddl");

  // popf rejects an unknown switch: no plan proves the argument got there
  ASSERT_FALSE(make_solver("args_unknown", {}, "-x")->getPlan(domain, problem, "args_test"));

  // Several arguments, one of them making popf more verbose
  auto plan = make_solver("args_several", {}, "-T -v1")->getPlan(domain, problem, "args_test");
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 3u);

  // The domain check does not use them: they are planning options
  auto checker = make_solver("args_check", {}, "-x");
  ASSERT_TRUE(checker->isDomainValid(read_test_file("domain_1_ok.pddl"), "args_test"));
}

TEST(popf_plan_solver, concurrent_runs_never_mix_their_files)
{
  // Two solvers, same output_dir and namespace, planning different problems at once
  const auto output_dir = fresh_folder("popf_concurrent");
  const auto domain = read_test_file("domain_simple.pddl");
  auto from_kitchen = make_solver("concurrent_1", output_dir);
  auto from_bedroom = make_solver("concurrent_2", output_dir);

  auto run = [&](std::shared_ptr<plansys2::POPFPlanSolver> solver, const std::string & room,
    size_t expected_size) {
      int wrong = 0;
      for (int i = 0; i < 15; i++) {
        auto plan = solver->getPlan(domain, problem_from(room), "/shared/ns");
        if (!plan || plan->items.size() != expected_size) {
          wrong++;
        }
      }
      return wrong;
    };
  auto kitchen = std::async(std::launch::async, run, from_kitchen, "kitchen", 3u);
  auto bedroom = std::async(std::launch::async, run, from_bedroom, "bedroom", 2u);
  ASSERT_EQ(kitchen.get(), 0);
  ASSERT_EQ(bedroom.get(), 0);

  std::filesystem::remove_all(output_dir);
}

TEST(popf_plan_solver, run_folders_are_removed_unless_kept)
{
  const auto output_dir = fresh_folder("popf_keep_files");
  const auto domain = read_test_file("domain_simple.pddl");
  const auto problem = read_test_file("problem_simple_1.pddl");
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("keep_files");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");
  node->set_parameter(rclcpp::Parameter("POPF.output_dir", output_dir.string()));

  const auto ns_dir = output_dir / "ns";
  ASSERT_TRUE(planner->getPlan(domain, problem, "ns"));
  ASSERT_TRUE(planner->isDomainValid(domain, "ns"));
  ASSERT_TRUE(std::filesystem::is_empty(ns_dir));

  node->set_parameter(rclcpp::Parameter("POPF.keep_files", true));
  ASSERT_TRUE(planner->getPlan(domain, problem, "ns"));
  std::vector<std::filesystem::path> runs(
    std::filesystem::directory_iterator(ns_dir), std::filesystem::directory_iterator{});
  ASSERT_EQ(runs.size(), 1u);
  for (const auto & file : {"domain.pddl", "problem.pddl", "plan"}) {
    ASSERT_TRUE(std::filesystem::exists(runs[0] / file)) << file;
  }

  std::filesystem::remove_all(output_dir);
}

TEST(popf_plan_solver, parse_plan_result_tolerates_unexpected_lines)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("parse_test");
  auto planner = std::make_shared<TestablePOPFPlanSolver>();
  planner->configure(node, "POPF");
  const auto folder = fresh_folder("popf_parse");
  std::filesystem::create_directories(folder);
  auto parse = [&](const std::string & content) {
      std::ofstream(folder / "plan") << content;
      return planner->parse_plan_result((folder / "plan").string());
    };

  // Empty lines, comments, warnings and malformed items around two good ones
  auto plan = parse(
    "Some header\n"
    "b (2.000 | 5.000);;;; Solution Found\n"
    "\n"
    "   \n"
    "; Time 0.00\n"
    "   ; indented comment\n"
    "0.000: (move leia kitchen bedroom)  [5.000]\n"
    "Warning: something unexpected\n"
    "0.000: (move leia\n"
    "abc: (approach leia bedroom jack)  [5.000]\n"
    "1.0: (approach leia bedroom jack)  []\n"
    "5.001: (approach leia bedroom jack) [5.000]\r\n"
    ")\n"
    "[\n");
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 2u);
  ASSERT_EQ(plan->items[0].action, "(move leia kitchen bedroom)");
  ASSERT_FLOAT_EQ(plan->items[0].duration, 5.0);
  ASSERT_EQ(plan->items[1].action, "(approach leia bedroom jack)");
  ASSERT_DOUBLE_EQ(plan->items[1].time, 5.001);

  // A number out of range is a bad line, not an exception; long times keep their decimals
  plan = parse(
    ";;;; Solution Found\n"
    "0.000: (move leia kitchen bedroom)  [1" + std::string(400, '0') + ".000]\n"
    "172800.003: (approach leia bedroom jack)  [86400.001]\n");
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->items.size(), 1u);
  ASSERT_DOUBLE_EQ(plan->items[0].time, 172800.003);
  ASSERT_DOUBLE_EQ(plan->items[0].duration, 86400.001);

  // A solution with no actions is an empty plan, not a failure
  plan = parse(";;;; Solution Found\n; Time 0.00\n");
  ASSERT_TRUE(plan);
  ASSERT_TRUE(plan->items.empty());

  // Without "Solution Found" there is no plan, whatever follows
  ASSERT_FALSE(parse("0.000: (move leia kitchen bedroom)  [5.000]\n"));
  ASSERT_FALSE(parse(""));
  ASSERT_FALSE(planner->parse_plan_result((folder / "missing").string()));

  std::filesystem::remove_all(folder);
}

// What popf declares is what it plans with: the planner gives it only those tasks
TEST(popf_plan_solver, supported_requirements_are_what_popf_plans_with)
{
  auto planner = make_solver("popf_requirements");
  const auto supported = planner->supported_requirements();
  EXPECT_TRUE(
    plansys2::missing_requirements(
      plansys2::pddl_requirements(
        read_test_file("domain_simple.pddl"), read_test_file("problem_simple_1.pddl")),
      supported).empty());

  // Each one alone, in a task popf solves without it
  const std::string domain =
    "(define (domain d) (:requirements :strips :negative-preconditions :conditional-effects)"
    " (:predicates (ready) (open) (lit))";
  const std::string problem = "(define (problem p) (:domain d) (:init (ready)) (:goal (lit)))";
  const std::vector<std::pair<std::string, std::string>> unsupported = {
    {":negative-preconditions",
      " (:action open-it :parameters () :precondition (not (open)) :effect (and (lit))))"},
    {":conditional-effects",
      " (:action light :parameters () :precondition (ready) :effect (when (ready) (lit))))"},
    {":disjunctive-preconditions",
      " (:action light :parameters () :precondition (or (ready) (open)) :effect (lit)))"},
  };
  for (const auto & [requirement, action] : unsupported) {
    EXPECT_EQ(
      plansys2::missing_requirements(
        plansys2::pddl_requirements(domain + action, problem), supported),
      std::vector<std::string>{requirement});
    EXPECT_FALSE(planner->getPlan(domain + action, problem, "popf_requirements")) << requirement;
  }
  EXPECT_TRUE(
    planner->getPlan(
      domain + " (:action light :parameters () :precondition (ready) :effect (lit)))", problem,
      "popf_requirements"));

  // Its parser rejects :action-costs; the same costs with :fluents plan
  const std::string costs =
    " (:predicates (ready) (lit)) (:functions (total-cost))"
    " (:action light :parameters () :precondition (ready)"
    "  :effect (and (lit) (increase (total-cost) 1))))";
  const std::string cost_problem =
    "(define (problem p) (:domain d) (:init (ready) (= (total-cost) 0)) (:goal (lit))"
    " (:metric minimize (total-cost)))";
  const std::string with_costs = "(define (domain d) (:requirements :strips :action-costs)" + costs;
  EXPECT_EQ(
    plansys2::missing_requirements(
      plansys2::pddl_requirements(with_costs, cost_problem), supported),
    std::vector<std::string>{":action-costs"});
  EXPECT_FALSE(planner->getPlan(with_costs, cost_problem, "popf_requirements"));
  const std::string with_fluents = "(define (domain d) (:requirements :strips :fluents)" + costs;
  EXPECT_TRUE(
    plansys2::missing_requirements(
      plansys2::pddl_requirements(with_fluents, cost_problem), supported).empty());
  EXPECT_TRUE(planner->getPlan(with_fluents, cost_problem, "popf_requirements"));
}

// The plan message as Kairos reads it: temporal if the domain has durative actions
kairos::Plan to_kairos(const plansys2_msgs::msg::Plan & msg, const std::string & domain)
{
  kairos::Plan plan;
  plan.temporal = domain.find(":durative-action") != std::string::npos;
  for (size_t i = 0; i < msg.items.size(); i++) {
    std::istringstream words(msg.items[i].action.substr(1, msg.items[i].action.size() - 2));
    kairos::PlanStep step;
    words >> step.name;
    for (std::string arg; words >> arg; ) {
      step.args.push_back(arg);
    }
    step.time = plan.temporal ? msg.items[i].time : static_cast<double>(i);
    step.duration = plan.temporal ? msg.items[i].duration : 0.0;
    plan.steps.push_back(step);
  }
  return plan;
}

bool valid(const parser::pddl::BankCase & task, const plansys2_msgs::msg::Plan & plan)
{
  kairos::ValidateOptions options;
  options.epsilon = 0.001;
  return kairos::validate_plan(
    task.domain, task.problem, to_kairos(plan, task.domain), options).valid();
}

// Every task of the bank: popf solves what it declares, and fails, at least once, on each
// requirement it does not
TEST(popf_plan_solver, the_bank)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  ASSERT_GE(cases.size(), 60u);
  auto planner = make_solver("popf_bank");
  const auto supported = planner->supported_requirements();

  std::set<std::string> excluded, justified;
  // popf aborts on preferences only when built with assertions (as in CI): not shown here
  justified.insert(":preferences");
  // On duration inequalities popf reads past the end of a vector: with assertions it aborts,
  // without them (the released package) it may still give a valid plan
  justified.insert(":duration-inequalities");
  int solved = 0;
  const auto all = std::chrono::steady_clock::now();
  for (const auto & task : cases) {
    SCOPED_TRACE(task.name);
    const auto missing = plansys2::missing_requirements(task.uses, supported);
    excluded.insert(missing.begin(), missing.end());
    bool pending = false;  // A requirement still without a case where popf fails
    for (const auto & requirement : missing) {
      pending = pending || !justified.count(requirement);
    }
    if (!missing.empty() && !pending) {
      continue;  // Enough: popf is not given these
    }

    const auto start = std::chrono::steady_clock::now();
    const auto plan = planner->getPlan(task.domain, task.problem, "popf_bank", 3s);
    const auto took = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
    EXPECT_LT(took.count(), 3.5);
    if (missing.empty()) {
      ASSERT_EQ(plan.has_value(), task.solvable);
      if (plan) {
        EXPECT_TRUE(valid(task, *plan));
        solved++;
      }
    } else if (task.solvable && !(plan && valid(task, *plan))) {
      justified.insert(missing.begin(), missing.end());
    }
  }
  EXPECT_EQ(justified, excluded);
  EXPECT_GE(solved, 15);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - all).count(), 40.0);
}

TEST(popf_plan_solver, unwritable_output_dir_fails_without_running_popf)
{
  const auto domain = read_test_file("domain_simple.pddl");
  const auto problem = read_test_file("problem_simple_1.pddl");

  // A read-only folder: no run folder can be created in it (root, as in CI, writes anyway)
  if (geteuid() != 0) {
    const auto read_only = fresh_folder("popf_read_only");
    std::filesystem::create_directories(read_only / "ns");
    std::filesystem::permissions(
      read_only / "ns", std::filesystem::perms::owner_read |
      std::filesystem::perms::owner_exec);
    auto planner = make_solver("read_only", read_only);
    ASSERT_FALSE(planner->getPlan(domain, problem, "ns"));
    ASSERT_FALSE(planner->isDomainValid(domain, "ns"));
    std::filesystem::permissions(read_only / "ns", std::filesystem::perms::owner_all);
    std::filesystem::remove_all(read_only);
  }

  // A file where a folder should be
  const auto file = fresh_folder("popf_not_a_folder");
  std::ofstream(file) << "not a folder\n";
  auto planner = make_solver("not_a_folder", file);
  ASSERT_FALSE(planner->getPlan(domain, problem, "ns"));
  ASSERT_FALSE(planner->isDomainValid(domain, "ns"));
  std::filesystem::remove(file);
}

TEST(popf_plan_solver, output_dir_without_home_or_namespace)
{
  const auto domain = read_test_file("domain_simple.pddl");
  const auto problem = read_test_file("problem_simple_1.pddl");

  // Without a namespace the output_dir itself is created
  const auto nested = fresh_folder("popf_no_ns") / "a" / "b";
  ASSERT_TRUE(make_solver("no_ns", nested)->getPlan(domain, problem, ""));
  ASSERT_TRUE(std::filesystem::is_directory(nested));
  std::filesystem::remove_all(nested.parent_path().parent_path());

  // HOME is only needed to expand ~
  const std::string home = std::getenv("HOME") ? std::getenv("HOME") : "";
  unsetenv("HOME");
  const auto plain = fresh_folder("popf_no_home");
  const bool plan_without_home = make_solver("no_home", plain)->getPlan(domain, problem,
    "").has_value();
  const bool tilde_without_home =
    make_solver("tilde_no_home", "~/popf_tilde")->getPlan(domain, problem, "").has_value();
  if (!home.empty()) {
    setenv("HOME", home.c_str(), 1);
  }
  std::filesystem::remove_all(plain);
  ASSERT_TRUE(plan_without_home);
  ASSERT_FALSE(tilde_without_home);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Before the process exits: Zenoh aborts if ROS is shut down by atexit
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
