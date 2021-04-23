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

#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <memory>
#include <tuple>

#include "plansys2_executor/behavior_tree/check_timeout_node.hpp"

namespace plansys2
{

CheckTimeout::CheckTimeout(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
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
CheckTimeout::tick()
{
  std::string action;
  getInput("action", action);

  if (status() == BT::NodeStatus::IDLE) {
    start_ = std::chrono::high_resolution_clock::now();
  }
  setStatus(BT::NodeStatus::RUNNING);

  if ((*action_map_)[action].action_executor != nullptr) {
    double duration = (*action_map_)[action].duration;
    double duration_overrun_percentage = (*action_map_)[action].duration_overrun_percentage;
    if (duration_overrun_percentage >= 0) {
      double max_duration = (1.0 + duration_overrun_percentage / 100.0) * duration;
      auto current_time = std::chrono::high_resolution_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - start_);
      if (elapsed_time > std::chrono::duration<double>(max_duration)) {
        std::cerr << "Actual duration of " << action << " exceeds max duration (" << std::fixed <<
          std::setprecision(2) << max_duration << " secs)." << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
