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

#include "plansys2_executor/ExecutorClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using ExecutePlan = plansys2_msgs::action::ExecutePlan;

ExecutorClient::ExecutorClient(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node)
{
  createActionClient();
}

void
ExecutorClient::createActionClient()
{
  action_client_ = rclcpp_action::create_client<ExecutePlan>(node_, "execute_plan");

  if (!this->action_client_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
  }
}

bool
ExecutorClient::start_plan_execution()
{
  if (!executing_plan_) {
    createActionClient();
    auto success = on_new_goal_received();

    if (success) {
      executing_plan_ = true;
      return true;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Already executing a plan");
  }

  return false;
}

bool
ExecutorClient::execute_and_check_plan()
{
  if (rclcpp::ok() && !goal_result_available_) {
    rclcpp::spin_some(node_);

    if (!goal_result_available_) {
      return true;  // Plan not finished
    }
  }

  switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Plan Succeded");
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(), "Plan Aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Plan Cancelled");
      break;

    default:
      throw std::logic_error("ExecutorClient::executePlan: invalid status value");
  }

  executing_plan_ = false;
  goal_result_available_ = false;

  return false;  // Plan finished
}


bool
ExecutorClient::on_new_goal_received()
{
  auto send_goal_options = rclcpp_action::Client<ExecutePlan>::SendGoalOptions();

  send_goal_options.feedback_callback =
    std::bind(&ExecutorClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&ExecutorClient::result_callback, this, _1);

  auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

  if (rclcpp::spin_until_future_complete(
      node_->get_node_base_interface(), future_goal_handle, 3s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    return false;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
    return false;
  }

  return true;
}

bool
ExecutorClient::should_cancel_goal()
{
  if (!executing_plan_) {
    return false;
  }

  rclcpp::spin_some(node_);
  auto status = goal_handle_->get_status();

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

void
ExecutorClient::cancel_plan_execution()
{
  if (should_cancel_goal()) {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (rclcpp::spin_until_future_complete(
        node_->get_node_base_interface(), future_cancel, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to cancel action server for execute_plan");
    }
  }

  executing_plan_ = false;
  goal_result_available_ = false;
}

void
ExecutorClient::feedback_callback(
  GoalHandleExecutePlan::SharedPtr goal_handle,
  const std::shared_ptr<const ExecutePlan::Feedback> feedback)
{
  feedback_ = *feedback;
}

void
ExecutorClient::result_callback(const GoalHandleExecutePlan::WrappedResult & result)
{
  goal_result_available_ = true;
  result_ = result;
  feedback_ = ExecutePlan::Feedback();
}

std::optional<ExecutePlan::Result>
ExecutorClient::getResult()
{
  if (result_.result != nullptr) {
    return *result_.result;
  } else {
    return {};
  }
}

}  // namespace plansys2
