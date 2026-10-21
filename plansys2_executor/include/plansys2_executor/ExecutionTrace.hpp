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

#ifndef PLANSYS2_EXECUTOR__EXECUTIONTRACE_HPP_
#define PLANSYS2_EXECUTOR__EXECUTIONTRACE_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "plansys2_msgs/msg/execution_event.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

/**
 * @brief What the executor does while a plan runs, and why, in order.
 *
 * Published on executor/execution_trace and, with log_execution_trace, one line each in the
 * log. The nodes of the tree find it in the blackboard as "trace".
 */
class ExecutionTrace
{
public:
  using Event = plansys2_msgs::msg::ExecutionEvent;

  ExecutionTrace(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    rclcpp_lifecycle::LifecyclePublisher<Event>::SharedPtr publisher, bool log);

  /// The start of the plan, from which plan_time counts
  void set_plan_start(const rclcpp::Time & start);

  /// The plan goes on later: its start, shifted by the late actions
  void shift_to(const rclcpp::Time & shifted_start);

  /// An event; planned_time only for START and END
  void event(
    uint8_t type, const std::string & action, const std::string & detail,
    double planned_time = 0.0);

  /// "START", "WAIT"...
  static std::string to_string(uint8_t type);

  /// The parts of conditions that do not hold, as text: why an action waits or fails
  static std::string unmet(
    const plansys2_msgs::msg::Tree & conditions,
    const std::function<bool(const plansys2_msgs::msg::Tree &)> & holds);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<Event>::SharedPtr publisher_;
  bool log_;
  std::mutex mutex_;
  uint64_t sequence_ {0};
  rclcpp::Time plan_start_;
  double shift_ {0.0};  // What the late actions moved it
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTIONTRACE_HPP_
