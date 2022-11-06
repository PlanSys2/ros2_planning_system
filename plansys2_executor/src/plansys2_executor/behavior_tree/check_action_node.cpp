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
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/check_action_node.hpp"

namespace plansys2
{

CheckAction::CheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
}

BT::NodeStatus
CheckAction::tick()
{
  std::string xml_action;
  getInput("action", xml_action);

  std::istringstream iss(xml_action);
  std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                  std::istream_iterator<std::string>{}};

  if (tokens.size() < 3) {
    return BT::NodeStatus::RUNNING;
  }

  auto action = tokens[0];
  auto lower = std::stod(tokens[1]);
  auto upper = std::stod(tokens[2]);

  if ((*action_map_).find(action) == (*action_map_).end()) {
    return BT::NodeStatus::RUNNING;  // Not started yet
  }

  if ((*action_map_)[action].action_executor != nullptr &&
    (*action_map_)[action].action_executor->is_finished() &&
    (*action_map_)[action].at_start_effects_applied &&
    (*action_map_)[action].at_end_effects_applied)
  {
    auto start_time = (*action_map_)[action].action_executor->get_start_time();
    auto current_time = (*action_map_)[action].action_executor->get_current_time();
    auto dt = current_time.seconds() - start_time.seconds();
    if (dt >= lower && dt < upper) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace plansys2
