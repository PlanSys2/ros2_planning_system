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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__RESTORE_ATSTART_EFFECT_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__RESTORE_ATSTART_EFFECT_NODE_HPP_

#include <map>
#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2
{

/**
 * @class plansys2::RestoreAtStartEffect
 * @brief BehaviorTree node that restores the at-start effects of a durative action.
 */
class RestoreAtStartEffect : public BT::ActionNodeBase
{
public:
  /**
   * @brief Constructor for the RestoreAtStartEffect node.
   *
   * @param[in] xml_tag_name Name of the XML tag for this node in the BT definition.
   * @param[in] conf Node configuration including the blackboard.
   */
  RestoreAtStartEffect(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  /**
   * @brief Halt method (required by the BT framework).
   */
  void halt() {}

  /**
   * @brief Main execution method for the BT node.
   *
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("action", "Action whose at end reqs must stop"),
      });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__RESTORE_ATSTART_EFFECT_NODE_HPP_
