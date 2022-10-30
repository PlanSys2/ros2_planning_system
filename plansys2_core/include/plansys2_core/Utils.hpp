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

#include <queue>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"

namespace plansys2
{
std::vector<std::string> tokenize(const std::string & string, const std::string & delim);

/**
 * @brief get a substring without empty lines
 *
 * @param string original string
 * @param init_pos first character in the original string
 * @param end_pos last character in the original string
 * @return a substring without empty lines
 */
std::string substr_without_empty_lines(
  std::string string, std::size_t init_pos, std::size_t end_pos);

/**
 * @brief remove the comments on a pddl string
 *
 * @param pddl a pddl string
 * @return a pddl string without comments
 */
std::string remove_comments(const std::string & pddl);

struct PlanNode
{
  plansys2_msgs::msg::PlanItem item;
  std::shared_ptr<PlanNode> true_node;
  std::shared_ptr<PlanNode> false_node;
};

/**
 * @brief convert PlanNode tree to plan message
 *
 * @param root of PlanNode tree
 * @return plan message
 */
plansys2_msgs::msg::Plan encode_plan(const std::shared_ptr<PlanNode> & root);

/**
 * @brief convert plan message to PlanNode tree
 *
 * @param plan message
 * @return root of PlanNode tree
 */
std::shared_ptr<PlanNode> decode_plan(const plansys2_msgs::msg::Plan & plan);
namespace internal
{
void encode_plan(
  const std::shared_ptr<PlanNode> & root, std::vector<int> & data_struct,
  std::vector<plansys2_msgs::msg::PlanItem> & data);

std::shared_ptr<PlanNode> decode_plan(
  std::vector<int> & data_struct, std::vector<plansys2_msgs::msg::PlanItem> & data);

std::shared_ptr<PlanNode> decode_plan(
  std::vector<int> & data_struct, std::vector<plansys2_msgs::msg::PlanItem> & data);

std::shared_ptr<PlanNode> decode_plan(
  std::queue<int> & data_struct, std::queue<plansys2_msgs::msg::PlanItem> & data);
}  // namespace internal

}  // namespace plansys2

#endif  // PLANSYS2_CORE__UTILS_HPP_
