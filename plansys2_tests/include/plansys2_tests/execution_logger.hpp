// Copyright 2021 Intelligent Robotics Lab
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

#ifndef PLANSYS2_TESTS__EXECUTION_LOGGER_HPP_
#define PLANSYS2_TESTS__EXECUTION_LOGGER_HPP_

#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_tests
{

class ExecutionLogger : public rclcpp::Node
{
public:
  using Ptr = std::shared_ptr<ExecutionLogger>;
  static Ptr make_shared()
  {
    return std::make_shared<ExecutionLogger>();
  }

  ExecutionLogger();

  // Copies: the subscriptions fill the logs from the thread that spins this node
  std::list<plansys2_msgs::msg::ActionExecution> get_action_execution_log() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return action_execution_log_;
  }

  std::list<plansys2_msgs::msg::ActionExecutionInfo> get_action_execution_info_log() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return action_execution_info_log_;
  }

  // Whether it hears the executor: before that, its messages are lost
  bool connected() const
  {
    return action_execution_info_sub_->get_publisher_count() > 0;
  }

  bool finished_before(const std::string & op1, const std::string & op2);
  bool started_before(const std::string & op1, const std::string & op2);
  bool before(const std::string & op1, const std::string & op2);
  bool sorted(const std::list<std::string> & actions);
  bool is_executed(const std::string & action_full_name);

  // The id the executor gives a plan item: "<action>:<start in ms>"
  static std::string id(const plansys2_msgs::msg::PlanItem & item);
  // The ids of the plan, in start order; with filter, only the actions that contain it
  static std::list<std::string> plan_order(
    const plansys2_msgs::msg::Plan & plan, const std::string & filter = "");

private:
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_execution_sub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr
    action_execution_info_sub_;

  mutable std::mutex mutex_;
  std::list<plansys2_msgs::msg::ActionExecution> action_execution_log_;
  std::list<plansys2_msgs::msg::ActionExecutionInfo> action_execution_info_log_;
};

}  // namespace plansys2_tests

#endif  // PLANSYS2_TESTS__EXECUTION_LOGGER_HPP_
