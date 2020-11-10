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

#include <memory>

#include "RepeatServer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_msgs/action/fibonacci.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

RepeaterServer::RepeaterServer()
: Node("fibonacci")
{
}

void
RepeaterServer::start_server()
{
  using namespace std::placeholders;

  fibonacci_action_server_ = rclcpp_action::create_server<Fibonacci>(
    shared_from_this(),
    "fibonacci",
    std::bind(&RepeaterServer::handle_goal, this, _1, _2),
    std::bind(&RepeaterServer::handle_cancel, this, _1),
    std::bind(&RepeaterServer::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Ready.");
}

rclcpp_action::GoalResponse
RepeaterServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Fibonacci::Goal> goal)
{
  if (goal->order > 0) {
    current_goal_ = *goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } else {
    return rclcpp_action::GoalResponse::REJECT;
  }
}

rclcpp_action::CancelResponse
RepeaterServer::handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
RepeaterServer::execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  rclcpp::Rate loop_rate(1);
  auto feedback = std::make_shared<Fibonacci::Feedback>();
  auto result = std::make_shared<Fibonacci::Result>();

  auto start = now();
  int current_times = 0;
  while (rclcpp::ok() && current_times < current_goal_.order) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Action Canceled");

      return;
    }

    RCLCPP_INFO(get_logger(), "Fibonaci %d / %d", current_times, current_goal_.order);

    feedback->sequence.push_back(current_times++);
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->sequence.push_back(current_times++);

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Action Succeeded");
  }
}

void
RepeaterServer::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&RepeaterServer::execute, this, _1), goal_handle}.detach();
}
