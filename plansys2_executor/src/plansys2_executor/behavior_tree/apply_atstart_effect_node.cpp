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
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/stn_dependency.hpp"

namespace plansys2
{

ApplyAtStartEffect::ApplyAtStartEffect(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
  // Only the STN builder needs these
  config().blackboard->get("action_graph", action_graph_);
  config().blackboard->get("bt_builder", bt_builder_);
  config().blackboard->get("node", node_);
  config().blackboard->get("stn_relaxed_timing", relaxed_timing_);
}

BT::NodeStatus
ApplyAtStartEffect::tick()
{
  std::string action;
  getInput("action", action);

  if (!(*action_map_)[action].at_start_effects_applied) {
    (*action_map_)[action].at_start_effects_applied = true;
    auto now = node_ ? node_->now() : rclcpp::Clock(RCL_ROS_TIME).now();
    (*action_map_)[action].at_start_effects_applied_time = now;
    // A non-durative action has no start effects, but its start still happens
    if (!(*action_map_)[action].action_info.is_action()) {
      apply((*action_map_)[action].action_info.get_at_start_effects(), problem_client_, 0);
    }

    // STNBTBuilder: the network takes the actual time of this event
    if (!relaxed_timing_ && !update_stn_on_event(
        action_graph_, bt_builder_, *action_map_, action, ActionType::START, now))
    {
      // Actual durations differ from the planned ones: the order still holds
      RCLCPP_WARN(
        rclcpp::get_logger("stn_bt"), "%s: the planned times no longer hold, keeping the order",
        action.c_str());
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
