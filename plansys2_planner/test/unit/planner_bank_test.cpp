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

// The planner with several combinations of Kairos, POPF and a solver that does not say what it
// plans with, on the tasks of the bank: who is given each task, and what comes back

#include <filesystem>
#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "kairos/kairos.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;

namespace
{

const char * kKairos = "plansys2/KairosPlanSolver";
const char * kPopf = "plansys2/POPFPlanSolver";

// Says nothing about what it plans with, and never finds a plan
class SilentSolver : public plansys2::PlanSolverBase
{
public:
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, const std::string &) override {}
  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string &, const std::string &, const std::string &,
    const rclcpp::Duration) override
  {
    calls++;
    return {};
  }
  bool isDomainValid(const std::string &, const std::string &) override {return true;}
  int calls = 0;
};

class TestPlannerNode : public plansys2::PlannerNode
{
public:
  plansys2_msgs::msg::PlanArray plan(
    const std::string & domain, const std::string & problem, std::string & error_info)
  {
    return solve(domain, problem, error_info);
  }
  void add_solver(const std::string & id, plansys2::PlanSolverBase::Ptr solver)
  {
    solvers_[id] = solver;
  }
  plansys2::PlanSolverBase::Ptr solver(const std::string & id) {return solvers_.at(id);}
};

struct Combination
{
  std::string name;
  std::vector<std::pair<std::string, std::string>> solvers;  // id, plugin
  bool silent = false;  // Plus a SilentSolver
  bool every_problem = true;  // Else only the first problem of each feature
};

kairos::Plan to_kairos(const plansys2_msgs::msg::Plan & msg, const std::string & domain)
{
  kairos::Plan plan;
  plan.temporal = domain.find(":durative-action") != std::string::npos || msg.has_end;
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
  if (msg.has_end) {
    plan.end = msg.end;
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

class PlannerBankTest : public ::testing::TestWithParam<Combination>
{
};

}  // namespace

TEST_P(PlannerBankTest, each_task_goes_to_the_solvers_that_plan_with_it)
{
  const auto & combination = GetParam();
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  ASSERT_GE(cases.size(), 60u);

  auto planner = std::make_shared<TestPlannerNode>();
  std::vector<std::string> ids;
  for (const auto & [id, plugin] : combination.solvers) {
    ids.push_back(id);
    planner->declare_parameter(id + ".plugin", plugin);
  }
  planner->set_parameter({"plan_solver_plugins", ids});
  planner->set_parameter({"plan_solver_timeout", 3.0});
  planner->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    planner->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  for (const auto & [id, plugin] : combination.solvers) {
    if (plugin == kKairos) {
      planner->set_parameter({id + ".anytime", false});  // The first plan is enough
    }
  }
  auto silent = std::make_shared<SilentSolver>();
  if (combination.silent) {
    planner->add_solver("SILENT", silent);
  }

  std::set<std::string> seen;
  int silent_calls = 0;
  const auto start = std::chrono::steady_clock::now();
  for (const auto & task : cases) {
    if (!combination.every_problem && !seen.insert(task.feature).second) {
      continue;
    }
    SCOPED_TRACE(task.name);

    // Who should get it, and how many of them solve it
    std::vector<std::string> chosen;
    for (const auto & id : ids) {
      if (plansys2::missing_requirements(
          task.uses, planner->solver(id)->supported_requirements()).empty())
      {
        chosen.push_back(id);
      }
    }
    // Kairos solves all it is given; POPF, what it says it plans with
    const size_t solving = task.solvable ? chosen.size() : 0;

    std::string error_info;
    const auto plans = planner->plan(task.domain, task.problem, error_info);
    for (const auto & plan : plans.plan_array) {
      EXPECT_TRUE(valid(task, plan));
    }
    if (chosen.empty() && !combination.silent) {
      // Nobody is given it, and the error says what each one lacks
      EXPECT_TRUE(plans.plan_array.empty());
      EXPECT_EQ(error_info.rfind("No solver plans with what the task uses: ", 0), 0u)
        << error_info;
      for (const auto & id : ids) {
        EXPECT_NE(error_info.find(id + " does not plan with"), std::string::npos) << error_info;
      }
    } else {
      EXPECT_EQ(plans.plan_array.size(), solving) << error_info;
      EXPECT_EQ(error_info.empty(), solving > 0) << error_info;
    }
    if (combination.silent) {
      EXPECT_EQ(silent->calls, ++silent_calls);  // It says nothing: it gets every task
    }
  }
  const auto took = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
  EXPECT_LT(took.count(), 30.0);
}

INSTANTIATE_TEST_SUITE_P(
  combinations, PlannerBankTest,
  ::testing::Values(
    Combination{"kairos", {{"KAIROS", kKairos}}},
    Combination{"popf", {{"POPF", kPopf}}, false, false},
    Combination{"kairos_and_popf", {{"KAIROS", kKairos}, {"POPF", kPopf}}},
    Combination{"popf_and_kairos", {{"A_POPF", kPopf}, {"B_KAIROS", kKairos}}},
    Combination{"two_kairos", {{"KAIROS", kKairos}, {"KAIROS_2", kKairos}}},
    Combination{"two_popf", {{"POPF", kPopf}, {"POPF_2", kPopf}}, false, false},
    Combination{"kairos_and_two_popf",
      {{"KAIROS", kKairos}, {"POPF", kPopf}, {"POPF_2", kPopf}}},
    Combination{"kairos_popf_and_silent", {{"KAIROS", kKairos}, {"POPF", kPopf}}, true}),
  [](const ::testing::TestParamInfo<Combination> & info) {return info.param.name;});

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
