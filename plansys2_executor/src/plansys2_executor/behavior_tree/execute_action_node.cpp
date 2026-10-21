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

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2
{

ExecuteAction::ExecuteAction(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
  std::ignore = config().blackboard->get("trace", trace_);
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
}

void
ExecuteAction::halt()
{
  // Halted by the BT (an over all requirement failed, or the plan stops): the performer stops
  std::string action;
  getInput("action", action);
  auto it = action_map_->find(action);
  if (it != action_map_->end() && it->second.action_executor) {
    auto status = it->second.action_executor->get_internal_status();
    if (status == ActionExecutor::RUNNING || status == ActionExecutor::DEALING) {
      it->second.action_executor->cancel();
    }
  }
}

BT::NodeStatus
ExecuteAction::tick()
{
  std::string action;
  getInput("action", action);

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  size_t delim = action.find(":");
  auto action_expr = action.substr(0, delim);

  if ((*action_map_)[action].action_executor == nullptr) {
    (*action_map_)[action].action_executor = ActionExecutor::make_shared(action_expr, node_);
  }

  auto & executor = (*action_map_)[action].action_executor;
  auto retval = executor->tick(node_->now());

  if (trace_ && executor->get_internal_status() == ActionExecutor::RUNNING &&
    !performing_reported_[action])
  {
    trace_->event(ExecutionTrace::Event::PERFORM, action, "by " + executor->get_performer());
    performing_reported_[action] = true;
  }
  if (retval == BT::NodeStatus::FAILURE) {
    (*action_map_)[action].execution_error_info = "Error executing the action";

    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "]" << (*action_map_)[action].execution_error_info);
    if (trace_) {
      const auto reason = executor->get_failure_reason();
      trace_->event(
        ExecutionTrace::Event::FAILURE, action,
        !reason.empty() ? reason :
        "its performer " + executor->get_performer() + " says: " + executor->get_feedback());
    }
  }

  return retval;
}

}  // namespace plansys2
