// Copyright 2025 Intelligent Robotics Lab
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

#include "plansys2_executor/behavior_tree/restore_atstart_effect_node.hpp"

namespace plansys2
{

RestoreAtStartEffect::RestoreAtStartEffect(
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
}

BT::NodeStatus
RestoreAtStartEffect::tick()
{
  std::string action;
  getInput("action", action);

  if ((*action_map_)[action].action_info.is_action()) {
    return BT::NodeStatus::FAILURE;
  }

  auto & info = (*action_map_)[action];
  if (info.at_start_effects_applied) {
    // Each fact goes back to its value before the action, unless someone else changed it
    undo(info.at_start_changes, problem_client_);
    info.at_start_changes.clear();
    info.at_start_effects_applied = false;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
