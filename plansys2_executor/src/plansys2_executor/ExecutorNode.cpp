// Copyright 2019 Intelligent Robotics Lab
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

#include "plansys2_executor/ExecutorNode.hpp"

#include <string>
#include <memory>
#include <iostream>
#include <fstream>

#include "plansys2_executor/ActionExecutor.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/action/execute_action.hpp"

namespace plansys2
{

using ExecutePlan = plansys2_msgs::action::ExecutePlan;

ExecutorNode::ExecutorNode()
: rclcpp_lifecycle::LifecycleNode("executor")
{
  using namespace std::placeholders;

  this->execute_plan_action_server_ = rclcpp_action::create_server<ExecutePlan>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "execute_plan",
    std::bind(&ExecutorNode::handle_goal, this, _1, _2),
    std::bind(&ExecutorNode::handle_cancel, this, _1),
    std::bind(&ExecutorNode::handle_accepted, this, _1));
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ExecutorNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  auto aux_node = std::make_shared<rclcpp::Node>("executor_helper");
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(aux_node);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(aux_node);
  planner_client_ = std::make_shared<plansys2::PlannerClient>(aux_node);

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

rclcpp_action::GoalResponse
ExecutorNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  auto start = now();
  RCLCPP_INFO(this->get_logger(), "Received goal request with order");

  auto domain = domain_client_->getDomain();
  auto problem = problem_client_->getProblem();

  auto domain_problem_ts = now();
  current_plan_ = planner_client_->getPlan(domain, problem);
  auto plan_ts = now();

  RCLCPP_INFO(get_logger(), "Getting domain and problem = %lf secs",
    (domain_problem_ts - start).seconds());

  RCLCPP_INFO(get_logger(), "Getting plan = %lf secs",
    (plan_ts - domain_problem_ts).seconds());

  if (current_plan_.has_value()) {
    std::cout << "plan: " << std::endl;
    for (const auto & action : current_plan_.value()) {
      std::cout << action.time << "\t" << action.action << "\t" <<
        action.duration << std::endl;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } else {
    RCLCPP_ERROR(get_logger(), "Executor problem [Plan not found]");
    return rclcpp_action::GoalResponse::REJECT;
  }
}

rclcpp_action::CancelResponse
ExecutorNode::handle_cancel(
  const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ExecutorNode::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  rclcpp::Rate loop_rate(10);
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  feedback->seq_action = 0;
  feedback->total_actions = current_plan_.value().size();
  for (const auto & action : current_plan_.value()) {
    feedback->seq_action++;
    feedback->current_action = action.action;

    auto action_executor = std::make_shared<ActionExecutor>(action.action);

    while (rclcpp::ok() && !action_executor->finished()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_info = "execution cancelled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }

      action_executor->update();

      feedback->progress_current_action = action_executor->getProgress();
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    auto status = action_executor->getStatus();
    if (status == ActionExecutor::AT_START_REQ_ERROR) {
      result->success = false;
      result->error_info = "Initial requirements of " + action.action + " not meet";
      goal_handle->succeed(result);
      return;
    } else if (status == ActionExecutor::OVER_ALL_REQ_ERROR) {
      result->success = false;
      result->error_info = "Over all requirements of " + action.action + " not meet";
      goal_handle->succeed(result);
      return;
    } else if (status == ActionExecutor::AT_END_REQ_ERROR) {
      result->success = false;
      result->error_info = "Over all requirements of " + action.action + " not meet";
      goal_handle->succeed(result);
      return;
    } else if (status == ActionExecutor::AT_START_EF_ERROR) {
      result->success = false;
      result->error_info = "At start effects of " + action.action + " could not be applied";
      goal_handle->succeed(result);
      return;
    } else if (status == ActionExecutor::AT_END_EF_ERROR) {
      result->success = false;
      result->error_info = "At end effects of " + action.action + " could not be applied";
      goal_handle->succeed(result);
      return;
    }
  }

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Plan Succeeded");
  }
}

void
ExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&ExecutorNode::execute, this, _1), goal_handle}.detach();
}

}  // namespace plansys2
