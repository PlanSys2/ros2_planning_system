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

#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>
#include <string>
#include <list>
#include <mutex>

#include "plansys2_tests/execution_logger.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2_tests
{

ExecutionLogger::ExecutionLogger()
: Node("execution_logger")
{
  action_execution_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub", 100,
    [this](plansys2_msgs::msg::ActionExecution::UniquePtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      action_execution_log_.push_back(std::move(*msg));
    });
  action_execution_info_sub_ = create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
    "/action_execution_info", 100,
    [this](plansys2_msgs::msg::ActionExecutionInfo::UniquePtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      action_execution_info_log_.push_back(std::move(*msg));
    });
}

bool
ExecutionLogger::finished_before(const std::string & op1, const std::string & op2)
{
  bool op1_found = false;
  bool op2_found = false;
  for (const auto & action : get_action_execution_info_log()) {
    op1_found = op1_found || (action.action_full_name == op1 &&
      action.completion > 0.9999999);
    op2_found = op2_found || (action.action_full_name == op2 &&
      action.completion > 0.9999999);

    if (op2_found && !op1_found) {
      std::cerr << op2 << " finished before " << op1 << std::endl;
      return false;
    }

    if (op1_found && op2_found) {
      break;
    }
  }

  return op1_found && op2_found;
}

bool
ExecutionLogger::started_before(const std::string & op1, const std::string & op2)
{
  bool op1_found = false;
  bool op2_found = false;
  for (const auto & action : get_action_execution_info_log()) {
    op1_found = op1_found || (action.action_full_name == op1 &&
      action.completion > 0.0000001);
    op2_found = op2_found || (action.action_full_name == op2 &&
      action.completion > 0.0000001);

    if (op2_found && !op1_found) {
      RCLCPP_ERROR_STREAM(get_logger(), op2 << " started before " << op1);
      return false;
    }

    if (op1_found && op2_found) {
      break;
    }
  }

  return op1_found && op2_found;
}

bool
ExecutionLogger::before(const std::string & op1, const std::string & op2)
{
  bool op1_found = false;
  bool op2_found = false;
  for (const auto & action : get_action_execution_info_log()) {
    op1_found = op1_found || (action.action_full_name == op1 &&
      action.completion > 0.9999999);
    op2_found = op2_found || (action.action_full_name == op2 &&
      action.completion > 0.0000001);

    if (op2_found && !op1_found) {
      RCLCPP_ERROR_STREAM(get_logger(), op2 << " started before " << op1 << " finishes");
      return false;
    }

    if (op1_found && op2_found) {
      break;
    }
  }

  return op1_found && op2_found;
}

bool
ExecutionLogger::is_executed(const std::string & action_full_name)
{
  bool executed = false;
  bool exist = false;
  for (const auto & action : get_action_execution_info_log()) {
    exist = exist || action.action_full_name == action_full_name;
    executed = executed || (action.action_full_name == action_full_name &&
      action.completion > 0.000001);
  }

  if (!exist) {
    RCLCPP_WARN_STREAM(get_logger(), action_full_name << " not exist (misspelled?) in plan");
  }

  return exist && executed;
}

bool
ExecutionLogger::sorted(const std::list<std::string> & actions)
{
  auto it = actions.begin();
  if (it == actions.end()) {
    return false;
  }

  auto it2 = std::next(it);
  while (it2 != actions.end()) {
    if (!is_executed(*it)) {
      RCLCPP_ERROR_STREAM(get_logger(), *it << " not found in execution");
      return false;
    }

    if (!is_executed(*it2)) {
      RCLCPP_ERROR_STREAM(get_logger(), *it2 << " not found in execution");
      return false;
    }

    // Started in order: STNBTBuilder may start an action before the previous one ends,
    // when it only needs what that one does at its start
    if (!started_before(*it, *it2)) {
      return false;
    }
    ++it;
    ++it2;
  }

  return true;
}


std::string
ExecutionLogger::id(const plansys2_msgs::msg::PlanItem & item)
{
  // Rounded, as the executor names its actions
  return item.action + ":" + std::to_string(std::llround(item.time * 1000));
}

std::list<std::string>
ExecutionLogger::plan_order(const plansys2_msgs::msg::Plan & plan, const std::string & filter)
{
  std::vector<plansys2_msgs::msg::PlanItem> items(plan.items.begin(), plan.items.end());
  std::stable_sort(
    items.begin(), items.end(),
    [](const auto & a, const auto & b) {return a.time < b.time;});

  std::list<std::string> ret;
  for (const auto & item : items) {
    if (filter.empty() || item.action.find(filter) != std::string::npos) {
      ret.push_back(id(item));
    }
  }
  return ret;
}

}  // namespace plansys2_tests
