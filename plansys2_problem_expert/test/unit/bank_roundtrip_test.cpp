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

// What the planner gets is what was loaded: every task of the bank, through the domain and
// problem experts, is the same task for Kairos (features, plans and metric)

#include <filesystem>
#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <string>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "kairos/kairos.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"

using namespace std::chrono_literals;

namespace
{
// Not representable yet: none
const std::set<std::string> kNotLoaded = {};

std::vector<parser::pddl::BankCase> bank()
{
  return parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
      "bank");
}

std::optional<kairos::Plan> solve(const std::string & domain, const std::string & problem)
{
  auto d = kairos::parse_domain(domain);
  EXPECT_TRUE(d) << domain;
  if (!d) {
    return {};
  }
  auto p = kairos::parse_problem(problem, *d);
  EXPECT_TRUE(p) << problem;
  if (!p) {
    return {};
  }
  kairos::SolveOptions options;
  options.timeout = 2s;
  options.anytime = false;
  return kairos::Planner().solve(*p, options).plan;
}

std::set<std::string> features(const std::string & domain, const std::string & problem)
{
  std::set<std::string> ret;
  auto d = kairos::parse_domain(domain);
  if (d) {
    for (const auto & f : d->features()) {
      ret.insert("domain " + f);
    }
    auto p = kairos::parse_problem(problem, *d);
    if (p) {
      for (const auto & f : p->features()) {
        ret.insert("problem " + f);
      }
    }
  }
  return ret;
}

kairos::ValidationReport check(
  const std::string & domain, const std::string & problem, const kairos::Plan & plan)
{
  kairos::ValidateOptions options;
  options.epsilon = 0.001;
  return kairos::validate_plan(domain, problem, plan, options);
}
}  // namespace

TEST(bank_roundtrip, the_planner_gets_the_task_that_was_loaded)
{
  const auto cases = bank();
  ASSERT_GE(cases.size(), 60u);
  int compared = 0;
  for (const auto & task : cases) {
    SCOPED_TRACE(task.name);
    // What PlanSys2 cannot represent is refused, in the domain or in the problem
    auto domain_expert = std::make_shared<plansys2::DomainExpert>(task.domain);
    plansys2::ProblemExpert problem_expert(domain_expert);
    const bool loaded = domain_expert->getParseError().empty() &&
      domain_expert->getUnsupported().empty() && problem_expert.addProblem(task.problem);
    EXPECT_EQ(loaded, !kNotLoaded.count(task.feature)) << domain_expert->getParseError();
    if (!loaded) {
      continue;
    }
    const auto domain = domain_expert->getDomain();
    const auto problem = problem_expert.getProblem();

    // The same features, read by Kairos
    EXPECT_EQ(features(domain, problem), features(task.domain, task.problem))
      << domain << "\n" << problem;

    // The same plans, both ways, with the same metric
    const auto original_plan = solve(task.domain, task.problem);
    const auto plan = solve(domain, problem);
    EXPECT_EQ(original_plan.has_value(), task.solvable);
    EXPECT_EQ(plan.has_value(), task.solvable) << domain << "\n" << problem;
    if (original_plan && plan) {
      const auto original_on_loaded = check(domain, problem, *original_plan);
      EXPECT_TRUE(original_on_loaded.valid()) << domain << "\n" << problem;
      EXPECT_TRUE(check(task.domain, task.problem, *plan).valid()) << domain << "\n" << problem;
      EXPECT_EQ(
        original_on_loaded.metric, check(task.domain, task.problem, *original_plan).metric);
    }
    compared++;
  }
  EXPECT_GE(compared, 40);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
