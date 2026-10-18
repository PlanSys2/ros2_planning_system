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
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
}

BT::NodeStatus
WaitAtStartReq::tick()
{
  std::string action;
  getInput("action", action);

  auto & info = (*action_map_)[action];
  if (info.action_executor != nullptr &&
    info.action_executor->get_internal_status() == ActionExecutor::RUNNING)
  {
    return BT::NodeStatus::SUCCESS;
  }

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  auto now = node->now();
  if (status() != BT::NodeStatus::RUNNING) {
    waiting_since_ = now;
    last_log_ = rclcpp::Time(0, 0, now.get_clock_type());
  }

  // PDDL+: what happens meanwhile depends on time, so an action starts at its planned time,
  // not before; its requirements are waited for from then on
  bool timed = false;
  rclcpp::Time plan_start;
  if (config().blackboard->get("timed_execution", timed) && timed &&
    config().blackboard->get("plan_start", plan_start) &&
    (now - plan_start).seconds() + 1e-3 < info.plan_item.time)
  {
    waiting_since_ = now;
    return BT::NodeStatus::RUNNING;
  }

  auto reqs_as = info.action_info.get_at_start_requirements();
  auto reqs_oa = info.action_info.get_overall_requirements();

  std::string unmet;
  const plansys2_msgs::msg::Tree * reqs = nullptr;
  if (!check_in_expert(reqs_as, problem_client_)) {
    unmet = "at start";
    reqs = &reqs_as;
  } else if (!check_in_expert(reqs_oa, problem_client_) && !holds_after_start(info, reqs_oa)) {
    unmet = "over all";
    reqs = &reqs_oa;
  } else {
    return BT::NodeStatus::SUCCESS;
  }

  info.execution_error_info = "Error checking " + unmet + " reqs";

  double timeout = 30.0;
  if (node->has_parameter("at_start_requirements_timeout")) {
    timeout = node->get_parameter("at_start_requirements_timeout").as_double();
  }
  const double waiting = (now - waiting_since_).seconds();
  if (timeout > 0.0 && waiting > timeout) {
    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "] Its " << unmet << " requirements did not hold in " << timeout <<
        " s: " << parser::pddl::toString(*reqs));
    return BT::NodeStatus::FAILURE;
  }

  // Waiting is normal for a while: not an error on every tick
  if ((now - last_log_).seconds() >= 5.0) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "[" << action << "] Waiting for its " << unmet << " requirements: " <<
        parser::pddl::toString(*reqs));
    last_log_ = now;
  }
  return BT::NodeStatus::RUNNING;
}

bool
WaitAtStartReq::holds_after_start(
  const ActionExecutionInfo & info, const plansys2_msgs::msg::Tree & reqs_oa)
{
  // Over all requirements hold once the action started: its at start effects may give them
  if (info.action_info.is_action()) {
    return false;
  }
  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();
  apply(info.action_info.get_at_start_effects(), predicates, functions);
  return check(reqs_oa, predicates, functions);
}

}  // namespace plansys2
