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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ATSTART_REQ_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ATSTART_REQ_NODE_HPP_

#include <map>
#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_executor/PredicateBTExecutor.hpp"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2
{

/**
 * @class plansys2::WaitAtStartReq
 * @brief BehaviorTree node that waits for the at-start requirements of a specified action.
 */
class WaitAtStartReq : public BT::ActionNodeBase
{
public:
  /**
   * @brief Constructor for the WaitAtStartReq node.
   *
   * @param[in] xml_tag_name Name of the XML tag for this node in the BT definition.
   * @param[in] conf Node configuration including the blackboard.
   */
  WaitAtStartReq(const std::string & xml_tag_name, const BT::NodeConfig & conf);

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
        BT::InputPort<std::string>("action", "Action whose at start reqs must stop"),
      });
  }

private:
  /**
   * @brief Checks if the at-start requirements are satisfied using a behavior tree.
   *
   * @param tree The PDDL tree to check against.
   * @param problem_client The ProblemExpertClient to use for checking.
   * @return true if the requirements are satisfied, false otherwise.
   */
  bool check_with_bt(
    const plansys2_msgs::msg::Tree & tree,
    const std::shared_ptr<plansys2::ProblemExpertClient> & problem_client);

  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::unique_ptr<PredicateBTExecutor> predicate_bt_executor_;
  rclcpp::Logger logger_{rclcpp::get_logger("WaitAtStartReq")};
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ATSTART_REQ_NODE_HPP_
