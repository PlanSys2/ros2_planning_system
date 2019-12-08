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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_

#include <string>
#include <memory>
#include <vector>

#include "std_msgs/msg/empty.hpp"
#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/Types.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

class ActionExecutorClient : public rclcpp::Node
{
public:
  using ExecuteAction = plansys2_msgs::action::ExecuteAction;
  using GoalHandleExecuteAction = rclcpp_action::ServerGoalHandle<ExecuteAction>;

  ActionExecutorClient(
    const std::string & action,
    float rate = 5);

protected:
  virtual void actionStep() = 0;
  virtual bool isFinished() = 0;

  virtual void onActivate() {}
  virtual void onFinish() {}

  std::shared_ptr<ExecuteAction::Feedback> getFeedback() {return feedback_;}
  const std::vector<std::string> & getArguments() const {return arguments_;}
  const std::string getName() const {return name_;}

private:
  std::shared_ptr<ExecuteAction::Feedback> feedback_;
  std::shared_ptr<ExecuteAction::Result> result_;
  std::vector<std::string> arguments_;

  rclcpp::Rate rate_;
  std::string name_;

  rclcpp_action::Server<ExecuteAction>::SharedPtr execute_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteAction> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteAction> goal_handle);

  void execute(const std::shared_ptr<GoalHandleExecuteAction> goal_handle);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
