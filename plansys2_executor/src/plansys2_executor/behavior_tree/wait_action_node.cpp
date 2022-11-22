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

#include "plansys2_executor/behavior_tree/wait_action_node.hpp"

namespace plansys2
{

WaitAction::WaitAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
}

BT::NodeStatus
WaitAction::tick()
{
  std::string xml_action;
  getInput("action", xml_action);

  auto p1 = xml_action.find(':') + 1;      // child time
  auto p2 = xml_action.find(' ', p1) + 1;  // child type
  auto p3 = xml_action.find(' ', p2) + 1;  // parent expression
  auto p4 = xml_action.find(':', p3) + 1;  // parent time
  auto p5 = xml_action.find(' ', p4) + 1;  // parent type
  auto p6 = xml_action.find(' ', p5) + 1;  // lower bound
  auto p7 = xml_action.find(' ', p6) + 1;  // upper bound

  auto child_id = xml_action.substr(0, p2 - 1);
  auto child_type = xml_action.substr(p2, p3 - p2 - 1);
  auto parent_id = xml_action.substr(p3, p5 - p3 - 1);
  auto parent_type = xml_action.substr(p5, p6 - p5 - 1);
  auto lower_str = xml_action.substr(p6, p7 - p6 - 1);
  auto upper_str = xml_action.substr(p7);
  auto lower = std::stod(lower_str);
  auto upper = std::stod(upper_str);

  if ((*action_map_).find(parent_id) == (*action_map_).end()) {
    return BT::NodeStatus::RUNNING;  // Not started yet
  }

  if ((*action_map_)[parent_id].action_executor == nullptr) {
    return BT::NodeStatus::RUNNING;
  }

  if (parent_type == "INIT" ||
      (parent_type == "START" && (*action_map_)[parent_id].at_start_effects_applied) ||
      (parent_type == "END" && (*action_map_)[parent_id].at_end_effects_applied &&
                               (*action_map_)[parent_id].action_executor->is_finished()))
  {
    if ((parent_id == child_id) && parent_type == "START" && child_type == "END") {
      return BT::NodeStatus::SUCCESS;
    }

    auto parent_time = (*action_map_)[parent_id].at_start_effects_applied_time;
    if (parent_type == "END") {
      parent_time = (*action_map_)[parent_id].at_end_effects_applied_time;
    }
    auto current_time = (*action_map_)[parent_id].action_executor->get_current_time();
    auto start_time = (*action_map_)[parent_id].action_executor->get_start_time();
    auto time_from_start = current_time.seconds() - start_time.seconds();
    auto dt = time_from_start - parent_time;

    if (dt >= lower && dt < upper) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace plansys2
