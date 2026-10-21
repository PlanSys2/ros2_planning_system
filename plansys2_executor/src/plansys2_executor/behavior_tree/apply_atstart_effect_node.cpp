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
  std::ignore = config().blackboard->get("trace", trace_);

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
  // Only the STN builder needs these: they may be missing
  std::ignore = config().blackboard->get("action_graph", action_graph_);
  std::ignore = config().blackboard->get("bt_builder", bt_builder_);
  std::ignore = config().blackboard->get("node", node_);
  std::ignore = config().blackboard->get("stn_relaxed_timing", relaxed_timing_);
}

BT::NodeStatus
ApplyAtStartEffect::tick()
{
  std::string action;
  getInput("action", action);

  auto & info = (*action_map_)[action];
  if (!info.at_start_effects_applied) {
    // A non-durative action has no start effects, but its start still happens
    if (!info.action_info.is_action()) {
      info.at_start_changes.clear();
      if (!plansys2::apply(
          info.action_info.get_at_start_effects(), problem_client_, info.at_start_changes))
      {
        // Half applied effects are taken back: the action does not start
        undo(info.at_start_changes, problem_client_);
        info.at_start_changes.clear();
        info.execution_error_info = "Error applying at start effects";
        if (trace_) {
          trace_->event(
            ExecutionTrace::Event::FAILURE, action, "its at start effects cannot apply");
        }
        return BT::NodeStatus::FAILURE;
      }
    }
    // Its continuous effects act from now on, integrated by the problem expert
    const auto continuous = info.action_info.is_empty() ?
      plansys2_msgs::msg::Tree() : info.action_info.get_continuous_effects();
    if (!continuous.nodes.empty() &&
      !problem_client_->setContinuousEffect(action, continuous, true))
    {
      undo(info.at_start_changes, problem_client_);
      info.at_start_changes.clear();
      info.execution_error_info = "Error starting its continuous effects";
      if (trace_) {
        trace_->event(
          ExecutionTrace::Event::FAILURE, action, "its continuous effects cannot start");
      }
      return BT::NodeStatus::FAILURE;
    }
    info.at_start_effects_applied = true;
    auto now = node_ ? node_->now() : rclcpp::Clock(RCL_ROS_TIME).now();
    info.at_start_effects_applied_time = now;
    if (trace_) {
      trace_->event(ExecutionTrace::Event::START, action, "", info.plan_item.time);
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
