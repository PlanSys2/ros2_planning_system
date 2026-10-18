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

#ifndef PLANSYS2_PDDL_PARSER__PDDLBANK_HPP_
#define PLANSYS2_PDDL_PARSER__PDDLBANK_HPP_

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace parser
{
namespace pddl
{

/**
 * @brief A task of the PDDL test bank: one feature per directory, a few problems each.
 *
 * The bank is installed with the tests (share/plansys2_pddl_parser/bank), so the tests of the
 * parser, the planner and the solver plugins check every feature on the same tasks.
 */
struct BankCase
{
  std::string feature;  // Its directory, "conditional-effects"
  std::string name;  // "conditional-effects/p2"
  std::string domain;  // The PDDL text
  std::string problem;
  std::filesystem::path domain_file;
  std::filesystem::path problem_file;
  bool solvable = true;
  std::set<std::string> uses;  // The requirements it uses, as pddl_requirements reads them
  std::string description;
};

inline std::string read_bank_file(const std::filesystem::path & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

/**
 * @brief Reads the bank: each <feature>/cases.tsv, one line per task, tab separated:
 * domain file, problem file, yes|no solvable, the requirements it uses ("-" if none), description.
 * @param[in] root The bank directory.
 * @return The tasks, by feature and problem; empty if there is no bank.
 */
inline std::vector<BankCase> read_pddl_bank(const std::filesystem::path & root)
{
  std::vector<BankCase> cases;
  std::error_code error;
  for (const auto & entry : std::filesystem::directory_iterator(root, error)) {
    const auto list = entry.path() / "cases.tsv";
    if (!std::filesystem::exists(list)) {
      continue;
    }
    std::ifstream in(list);
    std::string line;
    while (std::getline(in, line)) {
      std::vector<std::string> fields;
      std::stringstream row(line);
      std::string field;
      while (std::getline(row, field, '\t')) {
        fields.push_back(field);
      }
      if (fields.size() < 4) {
        continue;
      }
      BankCase task;
      task.feature = entry.path().filename().string();
      task.name = task.feature + "/" + std::filesystem::path(fields[1]).stem().string();
      task.domain_file = entry.path() / fields[0];
      task.problem_file = entry.path() / fields[1];
      task.domain = read_bank_file(task.domain_file);
      task.problem = read_bank_file(task.problem_file);
      task.solvable = fields[2] == "yes";
      std::stringstream uses(fields[3]);
      for (std::string requirement; uses >> requirement; ) {
        if (requirement != "-") {
          task.uses.insert(requirement);
        }
      }
      task.description = fields.size() > 4 ? fields[4] : "";
      cases.push_back(task);
    }
  }
  std::sort(
    cases.begin(), cases.end(),
    [](const BankCase & a, const BankCase & b) {return a.name < b.name;});
  return cases;
}

}  // namespace pddl
}  // namespace parser

#endif  // PLANSYS2_PDDL_PARSER__PDDLBANK_HPP_
