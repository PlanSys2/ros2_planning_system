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

/**
 * @class plansys2::CheckMethodPrecondition
 * @brief BehaviorTree node of a hierarchical (HTN) plan: whether the precondition of the
 * method that decomposes a task holds now, when the task starts. If not, the decomposition no
 * longer applies, and the task fails before any of its actions start.
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
      });
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<ExecutionTrace> trace_;  // Null outside an executor
  std::optional<plansys2_msgs::msg::Tree> precondition_;  // read once
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_METHOD_PRECONDITION_NODE_HPP_
