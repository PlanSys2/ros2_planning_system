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

#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"
#include "plansys2_executor/behavior_tree/stn_dependency.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

ApplyAtEndEffect::ApplyAtEndEffect(
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
ApplyAtEndEffect::tick()
{
  std::string action;
  getInput("action", action);

  auto effect = (*action_map_)[action].action_info.get_at_end_effects();

  auto & info = (*action_map_)[action];
  if (!info.at_end_effects_applied) {
    // PDDL+: what acts while it runs needs all of its planned duration, however soon its
    // performer is done
    bool timed = false;
    if (config().blackboard->get("timed_execution", timed) && timed &&
      info.action_info.is_durative_action() && info.at_start_effects_applied)
    {
      auto now = node_ ? node_->now() : rclcpp::Clock(RCL_ROS_TIME).now();
      if ((now - info.at_start_effects_applied_time).seconds() + 1e-3 < info.duration) {
        if (trace_ && !waiting_reported_) {
          trace_->event(
            ExecutionTrace::Event::WAIT, action,
            "for its end, after its planned duration: " + std::to_string(info.duration) + " s");
        }
        waiting_reported_ = true;
        return BT::NodeStatus::RUNNING;
      }
    }

    // Its continuous effects end with it
    if (!info.action_info.is_empty() &&
      !info.action_info.get_continuous_effects().nodes.empty())
    {
      problem_client_->setContinuousEffect(action, plansys2_msgs::msg::Tree(), false);
    }
    if (!plansys2::apply(effect, problem_client_, 0)) {
      info.execution_error_info = "Error applying at end effects";
      if (trace_) {
        trace_->event(ExecutionTrace::Event::FAILURE, action, "its at end effects cannot apply");
      }
      return BT::NodeStatus::FAILURE;
    }
    info.at_end_effects_applied = true;
    waiting_reported_ = false;
    if (trace_) {
      trace_->event(
        ExecutionTrace::Event::END, action, "", info.plan_item.time + info.plan_item.duration);
    }
    info.at_start_changes.clear();
    auto now = node_ ? node_->now() : rclcpp::Clock(RCL_ROS_TIME).now();
    info.at_end_effects_applied_time = now;

    // STNBTBuilder: the network takes the actual time of this event
    if (!relaxed_timing_ && !update_stn_on_event(
        action_graph_, bt_builder_, *action_map_, action, ActionType::END, now))
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
