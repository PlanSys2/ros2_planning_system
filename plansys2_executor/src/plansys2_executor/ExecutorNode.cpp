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

#include <filesystem>

#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>

#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

namespace plansys2
{

using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using namespace std::chrono_literals;

ExecutorNode::ExecutorNode()
: rclcpp_lifecycle::LifecycleNode("executor")
{
  using namespace std::placeholders;

#ifdef ZMQ_FOUND
  this->declare_parameter<bool>("enable_groot_monitoring", true);
  this->declare_parameter<int>("publisher_port", 2666);
  this->declare_parameter<int>("server_port", 2667);
  this->declare_parameter<int>("max_msgs_per_second", 25);
#endif

  execute_plan_action_server_ = rclcpp_action::create_server<ExecutePlan>(
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

  dotgraph_pub_ = this->create_publisher<std_msgs::msg::String>("dot_graph", 1);

  aux_node_ = std::make_shared<rclcpp::Node>("executor_helper");
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(aux_node_);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(aux_node_);
  planner_client_ = std::make_shared<plansys2::PlannerClient>(aux_node_);

  execution_info_pub_ = create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(
    "/action_execution_info", 100);

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  dotgraph_pub_->on_activate();
  execution_info_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  dotgraph_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  dotgraph_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  dotgraph_pub_.reset();
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
ExecutorNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  RCLCPP_DEBUG(this->get_logger(), "Received goal request with order");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ExecutorNode::handle_cancel(
  const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");

  cancel_plan_requested_ = true;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ExecutorNode::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  cancel_plan_requested_ = false;

  auto domain = domain_client_->getDomain();
  auto problem = problem_client_->getProblem();
  auto current_plan = planner_client_->getPlan(domain, problem);

  if (!current_plan.has_value()) {
    RCLCPP_ERROR(get_logger(), "No plan found");
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();

  for (const auto & action : current_plan.value()) {
    auto index = action.action + ":" + std::to_string(static_cast<int>(action.time * 1000));

    (*action_map)[index] = ActionExecutionInfo();
    (*action_map)[index].action_executor =
      ActionExecutor::make_shared(action.action, shared_from_this());
    (*action_map)[index].durative_action_info =
      get_action_from_string(action.action, domain_client_);
  }

  BTBuilder bt_builder(aux_node_);
  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", shared_from_this());
  blackboard->set("domain_client", domain_client_);
  blackboard->set("problem_client", problem_client_);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteAction>("ExecuteAction");
  factory.registerNodeType<WaitAction>("WaitAction");
  factory.registerNodeType<CheckOverAllReq>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReq>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReq>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffect>("ApplyAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffect>("ApplyAtEndEffect");

  auto bt_xml_tree = bt_builder.get_tree(current_plan.value());
  std_msgs::msg::String msg;
  msg.data =
    bt_builder.get_dotgraph(
    current_plan.value());
  dotgraph_pub_->publish(msg);

  std::filesystem::path tp = std::filesystem::temp_directory_path();
  std::ofstream out(std::string("/tmp/") + get_namespace() + "/bt.xml");
  out << bt_xml_tree;
  out.close();

  auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

#ifdef ZMQ_FOUND
  unsigned int publisher_port = this->get_parameter("publisher_port").as_int();
  unsigned int server_port = this->get_parameter("server_port").as_int();
  unsigned int max_msgs_per_second = this->get_parameter("max_msgs_per_second").as_int();

  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
  if (this->get_parameter("enable_groot_monitoring").as_bool()) {
    RCLCPP_INFO(
      get_logger(),
      "[%s] Groot monitoring: Publisher port: %d, Server port: %d, Max msgs per second: %d",
      get_name(), publisher_port, server_port, max_msgs_per_second);
    try {
      publisher_zmq.reset(
        new BT::PublisherZMQ(
          tree, max_msgs_per_second, publisher_port,
          server_port));
    } catch (const BT::LogicError & exc) {
      RCLCPP_ERROR(get_logger(), "ZMQ error: %s", exc.what());
    }
  }
#endif

  auto info_pub = create_wall_timer(
    1s, [this, &action_map]() {
      auto msgs = get_feedback_info(action_map);
      for (const auto & msg : msgs) {
        execution_info_pub_->publish(msg);
      }
    });

  rclcpp::Rate rate(10);
  auto start = now();
  auto status = BT::NodeStatus::RUNNING;

  while (status == BT::NodeStatus::RUNNING && !cancel_plan_requested_) {
    try {
      status = tree.tickRoot();
    } catch (std::exception & e) {
      std::cerr << e.what() << std::endl;
      status == BT::NodeStatus::FAILURE;
    }

    feedback->action_execution_status = get_feedback_info(action_map);
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  if (cancel_plan_requested_) {
    tree.haltTree();
  }

  if (status == BT::NodeStatus::FAILURE) {
    tree.haltTree();
    RCLCPP_ERROR(get_logger(), "Executor BT finished with FAILURE state");
  }

  result->success = status == BT::NodeStatus::SUCCESS;
  result->action_execution_status = get_feedback_info(action_map);

  size_t i = 0;
  while (i < result->action_execution_status.size() && result->success) {
    if (result->action_execution_status[i].status !=
      plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
    {
      result->success = false;
    }
    i++;
  }

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Plan Succeeded");
    } else {
      RCLCPP_INFO(this->get_logger(), "Plan Failed");
    }
  }
}

void
ExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&ExecutorNode::execute, this, _1), goal_handle}.detach();
}

std::vector<plansys2_msgs::msg::ActionExecutionInfo>
ExecutorNode::get_feedback_info(
  std::shared_ptr<std::map<std::string,
  ActionExecutionInfo>> action_map)
{
  std::vector<plansys2_msgs::msg::ActionExecutionInfo> ret;

  for (const auto & action : *action_map) {
    plansys2_msgs::msg::ActionExecutionInfo info;

    switch (action.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
      case ActionExecutor::DEALING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED;
        break;
      case ActionExecutor::RUNNING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
        break;
      case ActionExecutor::SUCCESS:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED;
        break;
      case ActionExecutor::FAILURE:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::FAILED;
        break;
      case ActionExecutor::CANCELLED:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
        break;
    }

    info.start_stamp = action.second.action_executor->get_start_time();
    info.status_stamp = action.second.action_executor->get_status_time();
    info.action = action.second.action_executor->get_action_name();

    info.arguments = action.second.action_executor->get_action_params();
    info.completion = action.second.action_executor->get_completion();
    info.message_status = action.second.action_executor->get_feedback();

    ret.push_back(info);
  }

  return ret;
}

void
ExecutorNode::print_execution_info(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info)
{
  fprintf(stderr, "Execution info =====================\n");

  for (const auto & action_info : *exec_info) {
    fprintf(stderr, "[%s]", action_info.first.c_str());
    switch (action_info.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
        fprintf(stderr, "\tIDLE\n");
        break;
      case ActionExecutor::DEALING:
        fprintf(stderr, "\tDEALING\n");
        break;
      case ActionExecutor::RUNNING:
        fprintf(stderr, "\tRUNNING\n");
        break;
      case ActionExecutor::SUCCESS:
        fprintf(stderr, "\tSUCCESS\n");
        break;
      case ActionExecutor::FAILURE:
        fprintf(stderr, "\tFAILURE\n");
        break;
    }
    if (action_info.second.durative_action_info == nullptr) {
      fprintf(stderr, "\tWith no duration info\n");
    }

    if (action_info.second.at_start_effects_applied) {
      fprintf(stderr, "\tAt start effects applied\n");
    } else {
      fprintf(stderr, "\tAt start effects NOT applied\n");
    }

    if (action_info.second.at_end_effects_applied) {
      fprintf(stderr, "\tAt end effects applied\n");
    } else {
      fprintf(stderr, "\tAt end effects NOT applied\n");
    }
  }
}

}  // namespace plansys2
