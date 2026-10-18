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

// Every task of the bank: what PlanSys2 supports parses and prints back the same, the rest
// fails with an error, never a crash

#include <set>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"

namespace
{
// Not read yet: none
const std::set<std::string> kNotRead = {};

std::vector<parser::pddl::BankCase> bank()
{
  return parser::pddl::read_pddl_bank(BANK_DIR);
}

template<typename T>
std::string print(const T & item)
{
  std::ostringstream out;
  out << item;
  return out.str();
}
}  // namespace

TEST(pddl_bank, has_every_feature)
{
  const auto cases = bank();
  std::set<std::string> features;
  for (const auto & task : cases) {
    features.insert(task.feature);
    EXPECT_FALSE(task.domain.empty()) << task.name;
    EXPECT_FALSE(task.problem.empty()) << task.name;
  }
  EXPECT_GE(features.size(), 24u);
  EXPECT_GE(cases.size(), 3 * features.size() - 4);
}

TEST(pddl_bank, supported_tasks_parse_and_print_back_the_same)
{
  for (const auto & task : bank()) {
    if (kNotRead.count(task.feature)) {
      continue;
    }
    SCOPED_TRACE(task.name);
    parser::pddl::Domain domain;
    ASSERT_NO_THROW(domain.parse(task.domain));
    parser::pddl::Instance problem(domain);
    ASSERT_NO_THROW(problem.parse(task.problem));

    // What it prints is read again as the same
    const auto domain_text = print(domain);
    parser::pddl::Domain again;
    ASSERT_NO_THROW(again.parse(domain_text)) << domain_text;
    EXPECT_EQ(print(again), domain_text);
    EXPECT_EQ(again.actions.size(), domain.actions.size());
    EXPECT_EQ(again.preds.size(), domain.preds.size());
    EXPECT_EQ(again.funcs.size(), domain.funcs.size());
    EXPECT_EQ(again.derived.size(), domain.derived.size());

    const auto problem_text = print(problem);
    parser::pddl::Instance problem_again(again);
    ASSERT_NO_THROW(problem_again.parse(problem_text)) << problem_text;
    EXPECT_EQ(print(problem_again), problem_text);
  }
}

TEST(pddl_bank, unsupported_tasks_fail_with_an_error)
{
  int checked = 0;
  for (const auto & task : bank()) {
    if (!kNotRead.count(task.feature)) {
      continue;
    }
    SCOPED_TRACE(task.name);
    bool failed = false;
    std::string error;
    try {
      parser::pddl::Domain domain;
      domain.parse(task.domain);
      parser::pddl::Instance problem(domain);
      problem.parse(task.problem);
    } catch (const std::exception & e) {
      failed = true;
      error = e.what();
    }
    EXPECT_TRUE(failed);
    EXPECT_FALSE(error.empty());
    checked++;
  }
  EXPECT_GE(checked, static_cast<int>(3 * kNotRead.size()));
}
