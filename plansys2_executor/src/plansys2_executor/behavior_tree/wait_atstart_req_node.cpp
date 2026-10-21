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

#include <cstdio>
#include <string>
#include <map>
#include <memory>
#include <tuple>
#include <algorithm>

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
  std::ignore = config().blackboard->get("trace", trace_);

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
    waiting_for_.clear();
  }
  auto wait = [&](const std::string & what) {
      if (trace_ && what != waiting_for_) {
        trace_->event(ExecutionTrace::Event::WAIT, action, what);
      }
      waiting_for_ = what;
    };

  // PDDL+: what happens meanwhile depends on time, so an action starts at its planned time,
  // not before; its requirements are waited for from then on
  bool timed = false;
  rclcpp::Time plan_start;
  if (config().blackboard->get("timed_execution", timed) && timed &&
    config().blackboard->get("plan_start", plan_start) &&
    (now - plan_start).seconds() + 1e-3 < info.plan_item.time)
  {
    waiting_since_ = now;
    char until[64];
    std::snprintf(until, sizeof(until), "for its time in the plan, %.3f", info.plan_item.time);
    wait(until);
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
    // Late: the rest of the plan shifts with it, so the planned intervals between actions hold.
    // Not what it was due before the plan started: that keeps the rest on the time of the world
    if (timed && !info.at_start_effects_applied) {
      rclcpp::Time execution_start = plan_start;
      std::ignore = config().blackboard->get("execution_start", execution_start);
      const double due_before = std::max(
        0.0, (execution_start - plan_start).seconds() - info.plan_item.time);
      const double late = (now - plan_start).seconds() - info.plan_item.time - due_before;
      if (late > 1e-3) {
        config().blackboard->set("plan_start", plan_start + rclcpp::Duration::from_seconds(late));
        if (trace_) {
          trace_->shift_to(plan_start + rclcpp::Duration::from_seconds(late));
          char detail[64];
          std::snprintf(detail, sizeof(detail), "%.3f s later: it starts late", late);
          trace_->event(ExecutionTrace::Event::SHIFT, action, detail);
        }
      }
    }
    return BT::NodeStatus::SUCCESS;
  }

  info.execution_error_info = "Error checking " + unmet + " reqs";

  double timeout = 30.0;
  if (node->has_parameter("at_start_requirements_timeout")) {
    timeout = node->get_parameter("at_start_requirements_timeout").as_double();
  }
  const double waiting = (now - waiting_since_).seconds();
  auto holds = [this](const plansys2_msgs::msg::Tree & part) {
      return check_in_expert(part, problem_client_);
    };
  if (timeout > 0.0 && waiting > timeout) {
    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "] Its " << unmet << " requirements did not hold in " << timeout <<
        " s: " << parser::pddl::toString(*reqs));
    if (trace_) {
      trace_->event(
        ExecutionTrace::Event::FAILURE, action,
        "its " + unmet + " requirements did not hold in " + std::to_string(timeout) + " s: " +
        ExecutionTrace::unmet(*reqs, holds));
    }
    return BT::NodeStatus::FAILURE;
  }
  if (trace_ && waiting_for_.rfind("for its " + unmet, 0) != 0) {
    wait("for its " + unmet + " requirements: " + ExecutionTrace::unmet(*reqs, holds));
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
  plansys2::apply(info.action_info.get_at_start_effects(), predicates, functions);
  return check(reqs_oa, predicates, functions);
}

}  // namespace plansys2
