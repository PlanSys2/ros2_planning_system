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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_METHOD_PRECONDITION_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_METHOD_PRECONDITION_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "plansys2_executor/ExecutionTrace.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2
{

/// @brief The blackboard entry that says that the method of task `id` was checked and holds.
inline std::string method_checked_key(const std::string & id) {return "htn_checked_" + id;}

/**
 * @class plansys2::CheckMethodPrecondition
 * @brief BehaviorTree node of a hierarchical (HTN) plan: whether the precondition of the
 * method that decomposes a task holds now, when the plan applied it. If not, the decomposition no
 * longer applies, and the task fails before any of its actions start. If it does, the actions
 * that start later in the plan may go on (WaitMethodCheck).
 */
class CheckMethodPrecondition : public BT::ActionNodeBase
{
public:
  CheckMethodPrecondition(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  void halt() override {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("task", "The task, \"(name args)\""),
        BT::InputPort<std::string>("method", "The method that decomposes it"),
        BT::InputPort<std::string>(
          "precondition", "Its precondition, in PDDL with the values of its parameters"),
        BT::InputPort<std::string>("id", "The task's id in the plan (Plan.tasks)"),
      });
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<ExecutionTrace> trace_;  // Null outside an executor
  std::optional<plansys2_msgs::msg::Tree> precondition_;  // read once
};

/**
 * @class plansys2::WaitMethodCheck
 * @brief BehaviorTree node of a hierarchical (HTN) plan: RUNNING until the precondition of the
 * method of task `id` has been checked, and held. An action that starts in the plan after the
 * method was applied waits so: it could change what the method needs before it is checked.
 */
class WaitMethodCheck : public BT::ActionNodeBase
{
public:
  WaitMethodCheck(const std::string & xml_tag_name, const BT::NodeConfig & conf)
  : ActionNodeBase(xml_tag_name, conf) {}

  void halt() override {}

  BT::NodeStatus tick() override
  {
    std::string id;
    getInput("id", id);
    bool checked = false;
    return config().blackboard->get(method_checked_key(id), checked) && checked ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({BT::InputPort<std::string>("id", "The task's id in the plan")});
  }
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_METHOD_PRECONDITION_NODE_HPP_
