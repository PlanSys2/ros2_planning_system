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

#ifndef PLANSYS2_EXECUTOR__REMAININGTASKS_HPP_
#define PLANSYS2_EXECUTOR__REMAININGTASKS_HPP_

#include <string>
#include <vector>

#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2
{

/**
 * @brief The initial task network (":htn") of what is left of a hierarchical plan, for a
 * replanning problem: the tasks of which no action has ended, whole (the planner may decompose
 * them again), and of those done in part, their subtasks; the actions not ended, as tasks. Each
 * keeps the orderings of the methods above it. A task with no actions goes in only if it comes
 * after something not done yet.
 *
 * @param plan A plan with its decomposition (Plan.tasks).
 * @param ended By item of the plan: whether its action has ended.
 * @return "(:htn ...)"; empty if the plan has no decomposition.
 */
std::string remaining_task_network(
  const plansys2_msgs::msg::Plan & plan, const std::vector<bool> & ended);

/// @brief `problem` with its ":htn" section replaced by `network` (or added before :init).
std::string with_task_network(const std::string & problem, const std::string & network);

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__REMAININGTASKS_HPP_
