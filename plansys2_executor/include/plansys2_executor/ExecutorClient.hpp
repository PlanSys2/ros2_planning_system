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

#ifndef PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_

#include <optional>
#include <string>
#include <memory>

#include "plansys2_msgs/action/execute_plan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

class ExecutorClient
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ClientGoalHandle<ExecutePlan>;

  explicit ExecutorClient(rclcpp::Node::SharedPtr provided_node);

  bool executePlan();

  ExecutePlan::Feedback getFeedBack() {return feedback_;}
  std::optional<ExecutePlan::Result> getResult();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecutePlan>::SharedPtr execute_plan_client_ptr_;

  ExecutePlan::Feedback feedback_;
  ExecutePlan::Feedback empty_feedback_;
  ExecutePlan::Result result_;
  bool finished_;

  void feedback_callback(
    GoalHandleExecutePlan::SharedPtr,
    const std::shared_ptr<const ExecutePlan::Feedback> feedback);

  void result_callback(const GoalHandleExecutePlan::WrappedResult & result);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
