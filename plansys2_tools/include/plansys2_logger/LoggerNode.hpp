// Copyright 2022 Intelligent Robotics Lab
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

#ifndef PLANSYS2_LOGGER__LOGGER_NODE_HPP_
#define PLANSYS2_LOGGER__LOGGER_NODE_HPP_

#include <memory>

#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2_logger
{

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode();

private:
  void knowledge_callback(plansys2_msgs::msg::Knowledge::SharedPtr msg);
  void action_execution_info_callback(plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg);
  void action_execution_callback(plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  rclcpp::Subscription<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_sub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr action_execution_info_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_execution_;
};

}  // namespace plansys2_logger

#endif  // PLANSYS2_LOGGER__LOGGER_NODE_HPP_
