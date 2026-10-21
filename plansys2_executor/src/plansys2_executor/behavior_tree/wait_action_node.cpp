// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <tuple>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/stn_dependency.hpp"

namespace plansys2
{

WaitAction::WaitAction(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
  std::ignore = config().blackboard->get("trace", trace_);
  // Only the STN builder needs these: they may be missing
  std::ignore = config().blackboard->get("action_graph", action_graph_);
  std::ignore = config().blackboard->get("node", node_);
  std::ignore = config().blackboard->get("stn_relaxed_timing", relaxed_timing_);
}

BT::NodeStatus
WaitAction::tick()
{
  std::string action;
  getInput("action", action);

  // STNBTBuilder: wait for the parent event and the time bounds between both
  if (auto dependency = parse_stn_dependency(action)) {
    if (dependency->parent_id == dependency->child_id) {
      return BT::NodeStatus::SUCCESS;  // Start and end of the same action
    }
    auto now = node_ ? node_->now() : rclcpp::Clock(RCL_ROS_TIME).now();
    auto elapsed = seconds_since_parent(*action_map_, dependency.value(), now);
    if (!elapsed) {
      if (trace_ && !waiting_reported_) {
        trace_->event(
          ExecutionTrace::Event::WAIT, dependency->child_id,
          "its " + dependency->child_type + " waits for the " + dependency->parent_type + " of " +
          dependency->parent_id);
      }
      waiting_reported_ = true;
      return BT::NodeStatus::RUNNING;
    }
    waiting_reported_ = false;
    auto [lower, upper] = stn_bounds(action_graph_, dependency.value());
    constexpr double kTolerance = 1e-3;
    if (!relaxed_timing_ && elapsed.value() < lower - kTolerance) {
      return BT::NodeStatus::RUNNING;  // Not before its planned time
    }
    if (!relaxed_timing_ && elapsed.value() > upper + kTolerance) {
      // Real durations are never exact: being late does not break the order, which is kept
      RCLCPP_WARN(
        rclcpp::get_logger("stn_bt"), "%s: %.3f s since %s %s, later than its window [%.3f, %.3f]",
        action.c_str(), elapsed.value(), dependency->parent_id.c_str(),
        dependency->parent_type.c_str(), lower, upper);
    }
    return BT::NodeStatus::SUCCESS;
  }

  if ((*action_map_).find(action) == (*action_map_).end()) {
    return BT::NodeStatus::RUNNING;  // Not started yet
  }

  if ((*action_map_)[action].action_executor != nullptr &&
    (*action_map_)[action].action_executor->is_finished())
  {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace plansys2
