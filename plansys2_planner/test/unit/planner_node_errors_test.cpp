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

// Planner errors are reported instead of killing or misconfiguring the process (#434).
// rclcpp is started with plan_solver_timeout:=2.0 for every node (see main).

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_plan_array.hpp"
#include "plansys2_msgs/srv/validate_domain.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace
{

std::string read_pddl(const std::string & name)
{
  // The POPF plugin's test files, with a problem POPF cannot finish quickly
  std::string path =
    plansys2::share_path("plansys2_popf_plan_solver").string() +
    "/pddl/" + name;
  std::ifstream ifs(path);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

class ThrowingSolver : public plansys2::PlanSolverBase
{
public:
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, const std::string &) override {}

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string &, const std::string &, const std::string &,
    const rclcpp::Duration) override
  {
    throw std::runtime_error("solver exploded");
  }

  bool isDomainValid(const std::string &, const std::string &) override
  {
    throw std::runtime_error("checker exploded");
  }
};

// Plans with what it says, counting its calls; its plan is one action, its name
class DeclaringSolver : public plansys2::PlanSolverBase
{
public:
  DeclaringSolver(std::string name, std::set<std::string> supported, bool finds = true)
  : name_(std::move(name)), supported_(std::move(supported)), finds_(finds) {}

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, const std::string &) override {}

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string &, const std::string &, const std::string &,
    const rclcpp::Duration) override
  {
    plans++;
    if (!finds_) {
      return {};
    }
    plansys2_msgs::msg::Plan plan;
    plan.items.emplace_back().action = "(" + name_ + ")";
    return plan;
  }

  bool isDomainValid(const std::string &, const std::string &) override
  {
    checks++;
    return true;
  }

  std::set<std::string> supported_requirements() const override {return supported_;}

  std::atomic<int> plans {0};
  std::atomic<int> checks {0};

private:
  std::string name_;
  std::set<std::string> supported_;
  bool finds_;
};

class TestPlannerNode : public plansys2::PlannerNode
{
public:
  void add_solver(const std::string & id, plansys2::PlanSolverBase::Ptr solver)
  {
    solvers_[id] = solver;
  }
  void remove_solver(const std::string & id) {solvers_.erase(id);}
};

// Log lines captured from every logger
std::mutex log_mutex;
std::vector<std::string> log_lines;

void capture_log(
  const rcutils_log_location_t *, int, const char * name, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  va_list copy;
  va_copy(copy, *args);
  char buffer[2048];
  vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  std::lock_guard<std::mutex> lock(log_mutex);
  log_lines.push_back(std::string(name) + ": " + buffer);
}

bool logged(const std::string & text)
{
  std::lock_guard<std::mutex> lock(log_mutex);
  for (const auto & line : log_lines) {
    if (line.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

class PlannerErrorsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_ = std::make_shared<TestPlannerNode>();
    client_node_ = rclcpp::Node::make_shared("planner_errors_client");
    get_plan_ = client_node_->create_client<plansys2_msgs::srv::GetPlan>("planner/get_plan");
    get_plan_array_ = client_node_->create_client<plansys2_msgs::srv::GetPlanArray>(
      "planner/get_plan_array");
    validate_ = client_node_->create_client<plansys2_msgs::srv::ValidateDomain>(
      "planner/validate_domain");
    exe_.add_node(planner_->get_node_base_interface());
    exe_.add_node(client_node_);
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  }

  void TearDown() override
  {
    spinner_.reset();
    planner_.reset();
  }

  void start_planner()
  {
    // These tests need a solver that takes its time on problem_hard_unsolvable, as POPF:
    // Kairos, the default, proves it unsolvable at once
    if (planner_->get_parameter("plan_solver_plugins").as_string_array().empty()) {
      planner_->set_parameter({"plan_solver_plugins", std::vector<std::string>{"POPF"}});
      plansys2::declare_parameter_if_not_declared(
        planner_, "POPF.plugin", rclcpp::ParameterValue(std::string("plansys2/POPFPlanSolver")));
    }
    planner_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    planner_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(planner_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(get_plan_->wait_for_service(5s));
  }

  template<typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> call(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    std::shared_ptr<typename ServiceT::Request> request)
  {
    auto future = client->async_send_request(request);
    if (future.wait_for(30s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> get_plan(
    const std::string & domain, const std::string & problem)
  {
    auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
    request->domain = domain;
    request->problem = problem;
    return call<plansys2_msgs::srv::GetPlan>(get_plan_, request);
  }

  std::shared_ptr<TestPlannerNode> planner_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_;
  rclcpp::Client<plansys2_msgs::srv::GetPlanArray>::SharedPtr get_plan_array_;
  rclcpp::Client<plansys2_msgs::srv::ValidateDomain>::SharedPtr validate_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

// A wrong plugin used to exit(-1) from on_configure
TEST_F(PlannerErrorsTest, unknown_solver_plugin_fails_configure)
{
  std::vector<std::string> solvers = {"GOOD", "BAD"};
  planner_->set_parameter({"plan_solver_plugins", solvers});
  for (const auto & id : solvers) {
    plansys2::declare_parameter_if_not_declared(
      planner_, id + ".plugin", rclcpp::ParameterValue(std::string()));
  }
  planner_->set_parameter({"GOOD.plugin", "plansys2/POPFPlanSolver"});
  planner_->set_parameter({"BAD.plugin", "plansys2/NoSuchSolver"});

  planner_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(planner_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // Fixed, the same node configures
  planner_->set_parameter({"BAD.plugin", "plansys2/POPFPlanSolver"});
  start_planner();
  auto response = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success);
}

TEST_F(PlannerErrorsTest, solver_exceptions_are_reported)
{
  start_planner();
  const auto domain = read_pddl("domain_simple.pddl");
  const auto problem = read_pddl("problem_simple_1.pddl");

  // Next to a good solver, its plan still comes back
  planner_->add_solver("THROWING", std::make_shared<ThrowingSolver>());
  auto response = get_plan(domain, problem);
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success);

  // Alone, it gives an error answer, not a dead planner
  planner_->remove_solver("POPF");
  response = get_plan(domain, problem);
  ASSERT_TRUE(response);
  ASSERT_FALSE(response->success);
  ASSERT_NE(response->error_info.find("THROWING failed: solver exploded"), std::string::npos)
    << response->error_info;

  auto array_request = std::make_shared<plansys2_msgs::srv::GetPlanArray::Request>();
  array_request->domain = domain;
  array_request->problem = problem;
  auto array_response = call<plansys2_msgs::srv::GetPlanArray>(get_plan_array_, array_request);
  ASSERT_TRUE(array_response);
  ASSERT_FALSE(array_response->success);
  ASSERT_NE(array_response->error_info.find("solver exploded"), std::string::npos);

  auto validate_request = std::make_shared<plansys2_msgs::srv::ValidateDomain::Request>();
  validate_request->domain = domain;
  auto validate_response = call<plansys2_msgs::srv::ValidateDomain>(validate_, validate_request);
  ASSERT_TRUE(validate_response);
  ASSERT_FALSE(validate_response->success);
  ASSERT_NE(validate_response->error_info.find("checker exploded"), std::string::npos);
}

TEST_F(PlannerErrorsTest, fractional_timeout_is_honored)
{
  // 1.5 s used to be truncated to 1 s
  planner_->set_parameter({"plan_solver_timeout", 1.5});
  start_planner();

  auto start = std::chrono::steady_clock::now();
  auto response = get_plan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_hard_unsolvable.pddl"));
  auto elapsed = std::chrono::steady_clock::now() - start;

  ASSERT_TRUE(response);
  ASSERT_FALSE(response->success);
  ASSERT_GE(elapsed, 1400ms);
  ASSERT_LT(elapsed, 4s);
  ASSERT_NE(response->error_info.find("POPF timed out after 1.5"), std::string::npos)
    << response->error_info;
}

TEST_F(PlannerErrorsTest, non_positive_timeout_uses_the_default)
{
  planner_->set_parameter({"plan_solver_timeout", 0.0});
  start_planner();
  ASSERT_TRUE(logged("plan_solver_timeout must be positive"));
  auto response = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success);
}

TEST_F(PlannerErrorsTest, error_info_tells_why_there_is_no_plan)
{
  start_planner();
  const auto domain = read_pddl("domain_simple.pddl");

  auto response = get_plan("(define (domain broken", read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(response);
  ASSERT_FALSE(response->success);
  ASSERT_EQ(response->error_info.rfind("PlanSys2 cannot read the domain", 0), 0u)
    << response->error_info;
  // And what the solver said too
  ASSERT_NE(response->error_info.find(". Plan not found: POPF found no plan"), std::string::npos)
    << response->error_info;

  response = get_plan(domain, "(define (problem broken) (:domain simple) (:objects");
  ASSERT_TRUE(response);
  ASSERT_FALSE(response->success);
  ASSERT_EQ(response->error_info.rfind("PlanSys2 cannot read the problem", 0), 0u)
    << response->error_info;

  // Valid PDDL with no plan
  response = get_plan(domain, read_pddl("problem_simple_2.pddl"));
  ASSERT_TRUE(response);
  ASSERT_FALSE(response->success);
  ASSERT_EQ(response->error_info, "Plan not found: POPF found no plan");

  response = get_plan(domain, read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success);
  ASSERT_TRUE(response->error_info.empty());
}

// The client used to give up exactly when the planner, with the same timeout, answered
TEST_F(PlannerErrorsTest, client_waits_for_the_planner_answer)
{
  // plan_solver_timeout 2.0, from the command line: for the planner and, set, for the client
  start_planner();
  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(client_node);

  auto plan = planner_client->getPlan(
    read_pddl("domain_simple.pddl"), read_pddl("problem_hard_unsolvable.pddl"));
  ASSERT_FALSE(plan);
  ASSERT_TRUE(logged("POPF timed out after 2"));
  ASSERT_FALSE(logged("service call timed out"));
}

// The planner gives a task only to the solvers that plan with what it uses
TEST_F(PlannerErrorsTest, solvers_are_chosen_by_what_the_task_uses)
{
  start_planner();
  planner_->remove_solver("POPF");
  const std::set<std::string> basic = {":strips", ":typing", ":durative-actions"};
  auto full = std::make_shared<DeclaringSolver>("full", plansys2::known_pddl_requirements());
  auto limited = std::make_shared<DeclaringSolver>("limited", basic);
  auto unknown = std::make_shared<DeclaringSolver>("unknown", std::set<std::string>{});
  planner_->add_solver("FULL", full);
  planner_->add_solver("LIMITED", limited);
  planner_->add_solver("UNKNOWN", unknown);

  const std::string plain =
    "(define (domain d) (:requirements :adl)"
    " (:predicates (p)) (:action a :parameters () :precondition (p) :effect (not (p))))";
  const std::string with_when =
    "(define (domain d) (:requirements :adl) (:predicates (p) (q))"
    " (:action a :parameters () :precondition (p) :effect (when (q) (not (p)))))";
  const std::string problem = "(define (problem p) (:domain d) (:init (p)) (:goal (not (p))))";

  // The goal is negated: LIMITED does not plan with it, the others do; :adl alone is not use
  auto array_request = std::make_shared<plansys2_msgs::srv::GetPlanArray::Request>();
  array_request->domain = plain;
  array_request->problem = "(define (problem p) (:domain d) (:init (p)) (:goal (and)))";
  auto array = call<plansys2_msgs::srv::GetPlanArray>(get_plan_array_, array_request);
  ASSERT_TRUE(array);
  EXPECT_EQ(array->plan_array.plan_array.size(), 3u);
  EXPECT_EQ(limited->plans, 1);

  array_request->problem = problem;
  array = call<plansys2_msgs::srv::GetPlanArray>(get_plan_array_, array_request);
  ASSERT_TRUE(array);
  EXPECT_EQ(array->plan_array.plan_array.size(), 2u);
  EXPECT_EQ(limited->plans, 1);
  EXPECT_EQ(full->plans, 2);
  EXPECT_EQ(unknown->plans, 2);

  // when: the same
  auto response = get_plan(with_when, problem);
  ASSERT_TRUE(response);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(limited->plans, 1);
  EXPECT_EQ(full->plans, 3);

  // The domain check goes to one that plans with it
  auto validate = std::make_shared<plansys2_msgs::srv::ValidateDomain::Request>();
  validate->domain = with_when;
  auto valid = call<plansys2_msgs::srv::ValidateDomain>(validate_, validate);
  ASSERT_TRUE(valid);
  EXPECT_TRUE(valid->success);
  EXPECT_EQ(limited->checks, 0);
  EXPECT_EQ(full->checks + unknown->checks, 1);
}

// With no solver for what the task uses, none is given it: they give wrong plans or crash
TEST_F(PlannerErrorsTest, with_no_solver_for_the_task_none_tries_and_the_error_says_why)
{
  start_planner();
  planner_->remove_solver("POPF");
  auto first = std::make_shared<DeclaringSolver>("first", std::set<std::string>{":strips"});
  auto second = std::make_shared<DeclaringSolver>(
    "second", std::set<std::string>{":strips", ":time"});
  planner_->add_solver("FIRST", first);
  planner_->add_solver("SECOND", second);

  const std::string domain =
    "(define (domain d) (:requirements :strips :negative-preconditions)"
    " (:predicates (p) (q)) (:derived (q) (not (p)))"
    " (:action a :parameters () :precondition (not (p)) :effect (p)))";
  const std::string problem = "(define (problem p) (:domain d) (:init) (:goal (p)))";
  auto response = get_plan(domain, problem);
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(first->plans, 0);
  EXPECT_EQ(second->plans, 0);
  EXPECT_EQ(
    response->error_info,
    "No solver plans with what the task uses: FIRST does not plan with :derived-predicates "
    ":negative-preconditions; SECOND does not plan with :derived-predicates "
    ":negative-preconditions");

  // Nor the domain check
  auto validate = std::make_shared<plansys2_msgs::srv::ValidateDomain::Request>();
  validate->domain = domain;
  auto valid = call<plansys2_msgs::srv::ValidateDomain>(validate_, validate);
  ASSERT_TRUE(valid);
  EXPECT_FALSE(valid->success);
  EXPECT_EQ(valid->error_info.rfind("No solver plans with what the task uses: FIRST", 0), 0u);
  EXPECT_EQ(first->checks + second->checks, 0);

  // Without anything special both plan
  response = get_plan(
    "(define (domain d) (:requirements :strips) (:predicates (p))"
    " (:action a :parameters () :precondition (and (p)) :effect (p)))", problem);
  ASSERT_TRUE(response);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(first->plans, 1);
  EXPECT_EQ(second->plans, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const char * args[] = {"test", "--ros-args", "-p", "plan_solver_timeout:=2.0"};
  rclcpp::init(4, args);
  rcutils_logging_set_output_handler(capture_log);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
