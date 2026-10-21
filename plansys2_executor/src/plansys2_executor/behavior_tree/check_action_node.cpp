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

#include <sstream>
#include <string>
#include <tuple>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/check_action_node.hpp"
#include "plansys2_executor/behavior_tree/stn_dependency.hpp"

namespace plansys2
{

CheckAction::CheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
  action_graph_ = config().blackboard->get<Graph::Ptr>("action_graph");
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  // Only with the STN builder: it may be missing
  std::ignore = config().blackboard->get("stn_relaxed_timing", relaxed_timing_);
}

BT::NodeStatus
CheckAction::tick()
{
  std::string xml_action;
  getInput("action", xml_action);

  auto dependency = parse_stn_dependency(xml_action);
  if (!dependency) {
    return BT::NodeStatus::FAILURE;  // Only the STN builder uses this node
  }
  if (dependency->parent_id == dependency->child_id &&
    dependency->parent_type == "START" && dependency->child_type == "END")
  {
    // An action's end always follows its own start
    auto started = seconds_since_parent(*action_map_, dependency.value(), node_->now());
    return started ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  auto elapsed = seconds_since_parent(*action_map_, dependency.value(), node_->now());
  if (!elapsed) {
    return BT::NodeStatus::RUNNING;  // The parent event has not happened yet
  }

  // The order is checked above; times only warn, as real durations are never exact
  constexpr double kTolerance = 1e-3;
  auto [lower, upper] = stn_bounds(action_graph_, dependency.value());
  if (!relaxed_timing_ &&
    (elapsed.value() < lower - kTolerance || elapsed.value() > upper + kTolerance))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("stn_bt"), "%s: %.3f s since %s %s, out of its window [%.3f, %.3f]",
      xml_action.c_str(), elapsed.value(), dependency->parent_id.c_str(),
      dependency->parent_type.c_str(), lower, upper);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
