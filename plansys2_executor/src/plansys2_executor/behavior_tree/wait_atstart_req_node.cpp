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
#include <tuple>

#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

WaitAtStartReq::WaitAtStartReq(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  logger_ = node->get_logger();

  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");

  predicate_bt_executor_ = std::make_unique<PredicateBTExecutor>(config().blackboard);
}

BT::NodeStatus
WaitAtStartReq::tick()
{
  std::string action;
  getInput("action", action);

  if (action_map_ != nullptr && (*action_map_)[action].action_executor != nullptr &&
    (*action_map_)[action].action_executor->get_internal_status() == ActionExecutor::RUNNING)
  {
    return BT::NodeStatus::SUCCESS;
  }

  auto reqs_as = (*action_map_)[action].action_info.get_at_start_requirements();
  auto reqs_oa = (*action_map_)[action].action_info.get_overall_requirements();

  bool check_as = check_with_bt(reqs_as, problem_client_);
  if (!check_as) {
    (*action_map_)[action].execution_error_info = "Error checking at start reqs";

    RCLCPP_ERROR_STREAM(
      logger_,
      "[" << action << "]" << (*action_map_)[action].execution_error_info << ": " <<
        parser::pddl::toString(reqs_as));

    return BT::NodeStatus::RUNNING;
  }

  bool check_oa = check_with_bt(reqs_oa, problem_client_);
  if (!check_oa) {
    (*action_map_)[action].execution_error_info = "Error checking over all reqs";

    RCLCPP_ERROR_STREAM(
      logger_,
      "[" << action << "]" << (*action_map_)[action].execution_error_info << ": " <<
        parser::pddl::toString(reqs_oa));

    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

bool WaitAtStartReq::check_with_bt(
  const plansys2_msgs::msg::Tree & tree,
  const std::shared_ptr<plansys2::ProblemExpertClient> & problem_client)
{
  std::function<bool(const plansys2_msgs::msg::Node &)> eval_node;
  eval_node = [&](const plansys2_msgs::msg::Node & node) -> bool
    {
      using Node = plansys2_msgs::msg::Node;
      switch (node.node_type) {
        case Node::AND:
          {
            for (const auto & child_id : node.children) {
              const auto & child = tree.nodes[child_id];
              if (!eval_node(child)) {
                return false;
              }
            }
            return true;
          }
        case Node::OR:
          {
            for (const auto & child_id : node.children) {
              const auto & child = tree.nodes[child_id];
              if (eval_node(child)) {
                return true;
              }
            }
            return false;
          }
        case Node::NOT:
          {
            const auto & child = tree.nodes[node.children[0]];
            return !eval_node(child);
          }
        case Node::PREDICATE:
          {
            return predicate_bt_executor_->check_predicate(node);
          }
        default:
          return false;
      }
    };

  if (tree.nodes.empty()) {
    return true;
  }

  const auto & root = tree.nodes[0];
  return eval_node(root);
}

}  // namespace plansys2
