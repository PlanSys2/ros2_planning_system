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

#include "plansys2_executor/ExecutionTrace.hpp"

#include <cstdio>
#include <memory>
#include <string>

#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

ExecutionTrace::ExecutionTrace(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  rclcpp_lifecycle::LifecyclePublisher<Event>::SharedPtr publisher, bool log)
: node_(node), publisher_(publisher), log_(log), plan_start_(node->now())
{
}

void
ExecutionTrace::set_plan_start(const rclcpp::Time & start)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_start_ = start;
  shift_ = 0.0;
}

void
ExecutionTrace::shift_to(const rclcpp::Time & shifted_start)
{
  std::lock_guard<std::mutex> lock(mutex_);
  shift_ = shifted_start.seconds() - plan_start_.seconds();
}

void
ExecutionTrace::event(
  uint8_t type, const std::string & action, const std::string & detail, double planned_time)
{
  Event msg;
  double shift = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.sequence = ++sequence_;
    const auto now = node_->now();
    msg.stamp = now;
    // Plain seconds: times of different clock types do not subtract
    msg.plan_time = now.seconds() - plan_start_.seconds();
    shift = shift_;
  }
  msg.type = type;
  msg.action = action;
  msg.planned_time = planned_time;
  msg.shift = shift;
  msg.detail = detail;
  if (publisher_ && publisher_->is_activated()) {
    publisher_->publish(msg);
  }
  if (log_) {
    char time[32];
    std::snprintf(time, sizeof(time), "%9.3f", msg.plan_time);
    std::string line = std::string("[trace] ") + time + " " + to_string(type);
    if (!action.empty()) {
      line += " " + action;
    }
    if (type == Event::START || type == Event::END) {
      // Late on the plan as the late actions before it shifted it
      char planned[96];
      if (shift > 1e-3) {
        std::snprintf(
          planned, sizeof(planned), " planned %.3f (+%.3f shifted) late %+.3f", planned_time,
          shift, msg.plan_time - planned_time - shift);
      } else {
        std::snprintf(
          planned, sizeof(planned), " planned %.3f late %+.3f", planned_time,
          msg.plan_time - planned_time);
      }
      line += planned;
    }
    if (!detail.empty()) {
      line += ": " + detail;
    }
    if (type == Event::FAILURE || type == Event::VIOLATION) {
      RCLCPP_WARN(node_->get_logger(), "%s", line.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "%s", line.c_str());
    }
  }
}

std::string
ExecutionTrace::unmet(
  const plansys2_msgs::msg::Tree & conditions,
  const std::function<bool(const plansys2_msgs::msg::Tree &)> & holds)
{
  std::string ret;
  for (const auto & part : parser::pddl::getSubtrees(conditions)) {
    if (!holds(part)) {
      ret += (ret.empty() ? "" : " ") + parser::pddl::toString(part);
    }
  }
  return ret.empty() ? parser::pddl::toString(conditions) : ret;
}

std::string
ExecutionTrace::to_string(uint8_t type)
{
  switch (type) {
    case Event::PLAN_START: return "PLAN_START";
    case Event::WAIT: return "WAIT";
    case Event::PERFORM: return "PERFORM";
    case Event::START: return "START";
    case Event::END: return "END";
    case Event::SHIFT: return "SHIFT";
    case Event::FAILURE: return "FAILURE";
    case Event::VIOLATION: return "VIOLATION";
    case Event::REPLAN: return "REPLAN";
    case Event::PLAN_END: return "PLAN_END";
  }
  return "UNKNOWN";
}

}  // namespace plansys2
