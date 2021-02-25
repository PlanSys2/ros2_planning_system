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

using ExecutePlan = plansys2_msgs::action::ExecutePlan;

ExecutorClient::ExecutorClient(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node), finished_(false)
{
  this->execute_plan_client_ptr_ = rclcpp_action::create_client<ExecutePlan>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "execute_plan");
}

bool
ExecutorClient::executePlan()
{
  using namespace std::placeholders;

  finished_ = false;

  if (!this->execute_plan_client_ptr_) {
    RCLCPP_ERROR(node_->get_logger(), "Action client not initialized");
  }

  if (!this->execute_plan_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return false;
  }

  auto goal_msg = ExecutePlan::Goal();

  auto send_goal_options = rclcpp_action::Client<ExecutePlan>::SendGoalOptions();

  send_goal_options.feedback_callback =
    std::bind(&ExecutorClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&ExecutorClient::result_callback, this, _1);
  auto goal_handle_future = this->execute_plan_client_ptr_->async_send_goal(
    goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      node_->get_logger(), "ExecutorClient: Plan execution was rejected by the action server");
    return false;
  }

  return true;
}

void
ExecutorClient::feedback_callback(
  GoalHandleExecutePlan::SharedPtr,
  const std::shared_ptr<const ExecutePlan::Feedback> feedback)
{
  feedback_ = *feedback;
}

void
ExecutorClient::result_callback(const GoalHandleExecutePlan::WrappedResult & result)
{
  finished_ = true;
  feedback_ = empty_feedback_;
  result_ = *result.result;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }

  if (result.result->success) {
    RCLCPP_INFO(node_->get_logger(), "Result received: Success");
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "Result received: Fail");
  }
}

std::optional<ExecutePlan::Result>
ExecutorClient::getResult()
{
  if (finished_) {
    return result_;
  } else {
    return {};
  }
}

}  // namespace plansys2
