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

#ifndef PLANSYS2_CORE__UTILS_HPP_
#define PLANSYS2_CORE__UTILS_HPP_

#include <string>
#include <vector>
#include <chrono>

namespace plansys2
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Splits a string into tokens based on a delimiter.
 * @param[in] string The original string to tokenize.
 * @param[in] delim The delimiter used to split the string.
 * @return std::vector<std::string> Vector containing the resulting tokens.
 */
std::vector<std::string> tokenize(const std::string & string, const std::string & delim);

/**
 * @brief Get a substring without empty lines.
 * @param[in] string The original string.
 * @param[in] init_pos The first character in the original string.
 * @param[in] end_pos The last character in the original string.
 * @return std::string A substring without empty lines.
 */
std::string substr_without_empty_lines(
  std::string string, std::size_t init_pos, std::size_t end_pos);

/**
 * @brief Remove the comments from a PDDL string.
 * @param[in] pddl A PDDL string.
 * @return std::string The PDDL string without comments.
 */
std::string remove_comments(const std::string & pddl);

/**
 * @brief Wait between tests for the middleware to forget the entities of the last one.
 *
 * It only waits: the nodes destroyed by a test take a while to disappear from discovery, and
 * the next test could see them.
 *
 * @param[in] total How long to wait.
 */
void drain_ros(std::chrono::milliseconds total = 200ms);

/**
 * @brief A node name that no other node has: the base, the process and a count.
 *
 * The clients of PlanSys2 are made in many processes, the PlanSys2 one among them: with the same
 * name, their service clients may not find the services they call for long.
 *
 * @param[in] base "problem_expert_client".
 * @return "problem_expert_client_4242_3".
 */
std::string unique_node_name(const std::string & base);

}  // namespace plansys2

#endif  // PLANSYS2_CORE__UTILS_HPP_
