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

#include "plansys2_executor/behavior_tree/check_method_precondition_node.hpp"

#include <memory>
#include <string>
#include <tuple>

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

CheckMethodPrecondition::CheckMethodPrecondition(
  const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  std::ignore = config().blackboard->get("trace", trace_);
  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

BT::NodeStatus
CheckMethodPrecondition::tick()
{
  std::string task, method, precondition, id;
  getInput("task", task);
  getInput("method", method);
  getInput("precondition", precondition);
  getInput("id", id);
  if (!precondition.empty() && !precondition_) {
    precondition_ = parser::pddl::fromString(precondition);
  }
  if (precondition.empty() || check_in_expert(*precondition_, problem_client_)) {
    // The actions after it in the plan may go on
    config().blackboard->set(method_checked_key(id), true);
    return BT::NodeStatus::SUCCESS;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::ignore = config().blackboard->get("node", node);
  RCLCPP_ERROR(
    node ? node->get_logger() : rclcpp::get_logger("htn_bt"),
    "[%s] Method %s no longer applies: its precondition does not hold: %s", task.c_str(),
    method.c_str(), precondition.c_str());
  if (trace_) {
    trace_->event(
      ExecutionTrace::Event::FAILURE, task,
      "the precondition of its method " + method + " does not hold: " + ExecutionTrace::unmet(
        *precondition_, [this](const plansys2_msgs::msg::Tree & part) {
          return check_in_expert(part, problem_client_);
        }));
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace plansys2
