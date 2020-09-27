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

#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"

namespace plansys2
{

CheckAtEndReq::CheckAtEndReq(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutor::Ptr>>>(
      "action_map");

  durative_actions_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, DurativeAction>>>(
      "action_info_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
      "problem_client");
}

BT::NodeStatus
CheckAtEndReq::tick()
{
  std::string action;
  getInput("action", action);
  
  size_t delim = action.find(":");
  auto action_expr = action.substr(0, delim);
  auto reqs = (*durative_actions_map_)[action_expr].at_end_requirements;

  if (!check(reqs.root_, problem_client_)) {
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2
