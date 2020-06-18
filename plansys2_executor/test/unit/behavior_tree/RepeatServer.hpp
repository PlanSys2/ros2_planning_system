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

#ifndef UNIT__BEHAVIOR_TREE__REPEATSERVER_HPP_
#define UNIT__BEHAVIOR_TREE__REPEATSERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_msgs/action/fibonacci.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RepeaterServer : public rclcpp::Node
{
public:
  using Fibonacci = test_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  RepeaterServer();

  void start_server();

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr fibonacci_action_server_;
  Fibonacci::Goal current_goal_;

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
};

#endif  // UNIT__BEHAVIOR_TREE__REPEATSERVER_HPP_
