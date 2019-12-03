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

#include "lifecycle_msgs/msg/state.hpp"

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
ExecutorNode::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with order");
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
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
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  
  auto & seq_action = feedback->seq_action;
  auto & total_action = feedback->total_actions;
  auto & current_action = feedback->current_action;
  auto & progress_current_action = feedback->progress_current_action;

  auto result = std::make_shared<ExecutePlan::Result>();

  total_action = 10;
  for (int i = 0; i < total_action && rclcpp::ok(); i++) {
    
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->error_info = "execution cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
      return;
    }

    seq_action = i;
    current_action = "action number " + std::to_string(i);
    progress_current_action =
      100.0f * static_cast<float>(seq_action) / static_cast<float>(total_action);
    
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }
}

void
ExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&ExecutorNode::execute, this, _1), goal_handle}.detach();
}

}  // namespace plansys2
