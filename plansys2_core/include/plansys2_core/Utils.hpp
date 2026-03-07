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
 * @brief Drain ROS messages for a given time. Useful in tests to ensure all messages are processed.
 * @param[in] n The node to use for spinning.
 * @param[in] total The total time to drain messages.
 */
void drain_ros(std::chrono::milliseconds total = 200ms);

}  // namespace plansys2

#endif  // PLANSYS2_CORE__UTILS_HPP_
