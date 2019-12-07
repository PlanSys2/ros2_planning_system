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

#include <string>
#include <memory>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

namespace plansys2
{

using ExecuteAction = plansys2_msgs::action::ExecuteAction;
using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;

ActionExecutorClient::ActionExecutorClient(
  const std::string & action,
  float rate)
: rclcpp::Node(action), rate_(rate), name_(action)
{
  using namespace std::placeholders;

  this->execute_action_server_ = rclcpp_action::create_server<ExecuteAction>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action,
    std::bind(&ActionExecutorClient::handle_goal, this, _1, _2),
    std::bind(&ActionExecutorClient::handle_cancel, this, _1),
    std::bind(&ActionExecutorClient::handle_accepted, this, _1));

  feedback_ = std::make_shared<ExecuteAction::Feedback>();
  result_ = std::make_shared<ExecuteAction::Result>();
}

rclcpp_action::GoalResponse
ActionExecutorClient::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received [%s] action request", goal->action.c_str());

  for (size_t i = 0; i < goal->arguments.size(); i++) {
    RCLCPP_INFO(this->get_logger(), " Argument %zu: [%s]", i, goal->arguments[i].c_str());
  }

  arguments_ = goal->arguments;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ActionExecutorClient::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel move action");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ActionExecutorClient::handle_accepted(const std::shared_ptr<GoalHandleExecuteAction> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&ActionExecutorClient::execute, this, _1), goal_handle}.detach();
}

void
ActionExecutorClient::execute(
  const std::shared_ptr<GoalHandleExecuteAction> goal_handle)
{
  feedback_ = std::make_shared<ExecuteAction::Feedback>();
  result_ = std::make_shared<ExecuteAction::Result>();

  feedback_->progress = 0.0;

  onActivate();

  while (rclcpp::ok() && !goal_handle->is_canceling() && !isFinished()) {
    actionStep();

    goal_handle->publish_feedback(feedback_);

    rate_.sleep();
  }

  if (goal_handle->is_canceling()) {
    result_->success = false;
    result_->error_info = "Charging action cancelled";
    goal_handle->canceled(result_);
  } else {
    result_->success = true;
    result_->error_info = "";
    goal_handle->succeed(result_);
  }

  onFinish();
}


}  // namespace plansys2
