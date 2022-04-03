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


#include <string>
#include <memory>
#include <iomanip>

#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_performer_status.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_logger/LoggerNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2_logger
{

using std::placeholders::_1;

LoggerNode::LoggerNode()
: Node("plansys2_logger")
{
  knowledge_sub_ = create_subscription<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge", rclcpp::QoS(100).transient_local(),
    std::bind(&LoggerNode::knowledge_callback, this, _1));

  action_execution_info_ = create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
    "action_execution_info", 100,
    std::bind(&LoggerNode::action_execution_info_callback, this, _1));

  action_execution_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable(),
    std::bind(&LoggerNode::action_execution_callback, this, _1));

  action_performer_status_ = create_subscription<plansys2_msgs::msg::ActionPerformerStatus>(
    "performers_status", rclcpp::QoS(100).reliable(),
    std::bind(&LoggerNode::action_performer_status_callback, this, _1));

  executing_plan_ = create_subscription<plansys2_msgs::msg::Plan>(
    "executing_plan", rclcpp::QoS(100).transient_local(),
    std::bind(&LoggerNode::executing_plan_callback, this, _1));
}

void
LoggerNode::knowledge_callback(plansys2_msgs::msg::Knowledge::SharedPtr msg)
{
  std::string goal_set = (msg->goal == "") ? "goal not set" : "goal set";
  RCLCPP_INFO_STREAM(
    get_logger(),
    "[Knowledge] " << msg->instances.size() << " instances -- " <<
      msg->predicates.size() << " predicates -- " <<
      msg->functions.size() << " functions -- " << goal_set);

  for (const std::string & instance : msg->instances) {
    RCLCPP_INFO_STREAM(get_logger(), "[Knowledge] Instance: " << instance);
  }

  for (const std::string & predicate : msg->predicates) {
    RCLCPP_INFO_STREAM(get_logger(), "[Knowledge] Predicate: " << predicate);
  }

  for (const std::string & function : msg->functions) {
    RCLCPP_INFO_STREAM(get_logger(), "[Knowledge] Function: " << function);
  }

  RCLCPP_INFO_STREAM(get_logger(), "[Knowledge] Goal: " << msg->goal);
}

void
LoggerNode::action_execution_info_callback(plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
{
  std::ostringstream output;

  std::string status = "UNKNOWN";
  switch (msg->status) {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      status = "NOT EXECUTED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
      status = "EXECUTING";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      status = "SUCCEEDED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      status = "CANCELLED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      status = "FAILED";
      break;
  }

  output << std::fixed << std::setprecision(5);
  output << "[Action Execution Info] Action: " << msg->action_full_name <<
    " -- Expected time: " << rclcpp::Duration(msg->duration).seconds() << std::endl;
  output << "status: " << status << std::endl;
  output << " [" << msg->action << "]";
  for (const auto & arg : msg->arguments) {
    output << " (" << arg << ")";
  }
  output << std::endl;
  output << "Start time: " << rclcpp::Time(msg->start_stamp).seconds() <<
    " -- Status time: " << rclcpp::Time(msg->status_stamp).seconds() << std::endl;
  output << "Completion: " << msg->completion <<
    " -- Status message: " << msg->status << std::endl;

  switch (msg->status) {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      RCLCPP_DEBUG_STREAM(get_logger(), output.str());
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      RCLCPP_INFO_STREAM(get_logger(), output.str());
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      RCLCPP_WARN_STREAM(get_logger(), output.str());
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      RCLCPP_ERROR_STREAM(get_logger(), output.str());
      break;
  }
}

void
LoggerNode::action_execution_callback(plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  std::ostringstream output;

  std::string status = "UNKNOWN";
  switch (msg->type) {
    case plansys2_msgs::msg::ActionExecution::REQUEST:
      {
        std::ostringstream output;
        output << "[Action Hub] REQUEST action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        output << "]";
        RCLCPP_INFO_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
      {
        std::ostringstream output;
        output << "[Action Hub] RESPONSE action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        output << "] from " << msg->node_id;
        RCLCPP_INFO_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
      {
        std::ostringstream output;
        output << "[Action Hub] CONFIRM action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        output << "] to " << msg->node_id;
        RCLCPP_INFO_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionExecution::REJECT:
      {
        std::ostringstream output;
        output << "[Action Hub] REJECT action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        output << "] to " << msg->node_id;
        RCLCPP_INFO_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
      {
        std::ostringstream output;
        output << "[Action Hub] FEEDBACK action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        output << "] from " << msg->node_id << " -- completion: " << msg->completion <<
          " [" << msg->status << "]";
        RCLCPP_INFO_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionExecution::FINISH:
      {
        std::ostringstream output;
        output << "[Action Hub] FINISH action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        if (msg->success) {
          output << "] from " << msg->node_id << " -- Success  [" << msg->status << "]";
          RCLCPP_INFO_STREAM(get_logger(), output.str());
        } else {
          output << "] from " << msg->node_id << " -- Failed  [" << msg->status << "]";
          RCLCPP_WARN_STREAM(get_logger(), output.str());
        }
      }
      break;
    case plansys2_msgs::msg::ActionExecution::CANCEL:
      {
        std::ostringstream output;
        output << "[Action Hub] CANCEL action: [" << msg->action << "]";
        for (const auto & arg : msg->arguments) {
          output << " (" << arg << ")";
        }
        output << "] to " << msg->node_id;
        RCLCPP_WARN_STREAM(get_logger(), output.str());
      }
      break;
  }
}

void
LoggerNode::action_performer_status_callback(
  plansys2_msgs::msg::ActionPerformerStatus::SharedPtr msg)
{
  switch (msg->state) {
    case plansys2_msgs::msg::ActionPerformerStatus::NOT_READY:
      {
        std::ostringstream output;
        output << "[Performers] " << msg->node_name << " for action " << msg->action;
        for (const auto & arg : msg->specialized_arguments) {
          output << "(specialized in " << arg << ")";
        }
        output << " NOT READY ";
        RCLCPP_WARN_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionPerformerStatus::READY:
      {
        std::ostringstream output;
        output << "[Performers] " << msg->node_name << " for action " << msg->action;
        for (const auto & arg : msg->specialized_arguments) {
          output << "(specialized in " << arg << ")";
        }
        output << " READY ";
        RCLCPP_DEBUG_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionPerformerStatus::RUNNING:
      {
        std::ostringstream output;
        output << "[Performers] " << msg->node_name << " for action " << msg->action;
        for (const auto & arg : msg->specialized_arguments) {
          output << "(specialized in " << arg << ")";
        }
        output << " RUNNING ";
        RCLCPP_INFO_STREAM(get_logger(), output.str());
      }
      break;
    case plansys2_msgs::msg::ActionPerformerStatus::FAILURE:
      {
        std::ostringstream output;
        output << "[Performers] " << msg->node_name << " for action " << msg->action;
        for (const auto & arg : msg->specialized_arguments) {
          output << "(specialized in " << arg << ")";
        }
        output << " FAILURE ";
        RCLCPP_WARN_STREAM(get_logger(), output.str());
      }
      break;
  }
}

void
LoggerNode::executing_plan_callback(plansys2_msgs::msg::Plan::SharedPtr msg)
{
  if (msg->items.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "[Executing Plan] No plan for execution");
  } else {
    std::ostringstream output;
    output << "[Executing Plan] Executing Plan with " << msg->items.size() <<
      " actions" << std::endl;
    for (const auto & item : msg->items) {
      output << "\t" << item.action << " [" << item.time << ", " << item.duration << "]" <<
        std::endl;
    }
    RCLCPP_INFO_STREAM(get_logger(), output.str());
  }
}

}  // namespace plansys2_logger
