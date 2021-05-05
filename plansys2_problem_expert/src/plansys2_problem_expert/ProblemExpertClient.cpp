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

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_pddl_parser/Utils.h"

namespace plansys2
{

ProblemExpertClient::ProblemExpertClient(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node)
{
  add_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::AddProblemGoal>(
    "problem_expert/add_problem_goal");
  add_problem_instance_client_ = node_->create_client<plansys2_msgs::srv::AffectParam>(
    "problem_expert/add_problem_instance");
  add_problem_predicate_client_ = node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_predicate");
  add_problem_function_client_ = node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_function");
  get_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::GetProblemGoal>(
    "problem_expert/get_problem_goal");
  get_problem_instance_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "problem_expert/get_problem_instance");
  get_problem_instances_client_ = node_->create_client<plansys2_msgs::srv::GetProblemInstances>(
    "problem_expert/get_problem_instances");
  get_problem_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_predicate");
  get_problem_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_predicates");
  get_problem_function_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_function");
  get_problem_functions_client_ = node_->create_client<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_functions");
  get_problem_client_ = node_->create_client<plansys2_msgs::srv::GetProblem>(
    "problem_expert/get_problem");
  remove_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::RemoveProblemGoal>(
    "problem_expert/remove_problem_goal");
  clear_problem_knowledge_client_ = node_->create_client<plansys2_msgs::srv::ClearProblemKnowledge>(
    "problem_expert/clear_problem_knowledge");
  remove_problem_instance_client_ =
    node_->create_client<plansys2_msgs::srv::AffectParam>(
    "problem_expert/remove_problem_instance");
  remove_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_predicate");
  remove_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_function");
  exist_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_predicate");
  exist_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_function");
  update_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/update_problem_function");
  is_problem_goal_satisfied_client_ =
    node_->create_client<plansys2_msgs::srv::IsProblemGoalSatisfied>(
    "problem_expert/is_problem_goal_satisfied");
}

std::vector<plansys2::Instance>
ProblemExpertClient::getInstances()
{
  auto params = getInstanceParams();
  std::vector<plansys2::Instance> ret;
  ret.reserve(params.size());
  std::transform(
    params.begin(), params.end(), std::back_inserter(ret),
    [](plansys2_msgs::msg::Param item)
    {
      return plansys2::Instance(item);
    });
  return ret;
}

std::vector<plansys2_msgs::msg::Param>
ProblemExpertClient::getInstanceParams()
{
  while (!get_problem_instances_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instances_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstances::Request>();

  auto future_result = get_problem_instances_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->instances;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      node_->get_namespace() <<
        get_problem_instances_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addInstance(const plansys2::Instance & instance)
{
  return addInstanceParam(instance);
}

bool
ProblemExpertClient::addInstanceParam(const plansys2_msgs::msg::Param & instance)
{
  while (!add_problem_instance_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_instance_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  request->param = instance;

  auto future_result = add_problem_instance_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_instance_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::removeInstance(const plansys2::Instance & instance)
{
  return removeInstanceParam(instance);
}

bool
ProblemExpertClient::removeInstanceParam(const plansys2_msgs::msg::Param & instance)
{
  while (!remove_problem_instance_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_instance_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  request->param = instance;

  auto future_result = remove_problem_instance_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_instance_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

std::optional<plansys2::Instance>
ProblemExpertClient::getInstance(const std::string & name)
{
  return getInstanceParam(name);
}

std::optional<plansys2_msgs::msg::Param>
ProblemExpertClient::getInstanceParam(const std::string & name)
{
  while (!get_problem_instance_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instance_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstanceDetails::Request>();

  request->instance = name;

  auto future_result = get_problem_instance_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->instance;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instance_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

std::vector<plansys2::Predicate>
ProblemExpertClient::getPredicates()
{
  auto predicates = getPredicateNodes();
  std::vector<plansys2::Predicate> ret;
  ret.reserve(predicates.size());
  std::transform(
    predicates.begin(), predicates.end(), std::back_inserter(ret),
    [](plansys2_msgs::msg::Node item)
    {
      return plansys2::Predicate(item);
    });
  return ret;
}

std::vector<plansys2_msgs::msg::Node>
ProblemExpertClient::getPredicateNodes()
{
  while (!get_problem_predicates_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto future_result = get_problem_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->states;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addPredicate(const plansys2::Predicate & predicate)
{
  return addPredicateNode(predicate);
}

bool
ProblemExpertClient::addPredicateNode(const plansys2_msgs::msg::Node & predicate)
{
  while (!add_problem_predicate_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_predicate_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = predicate;

  auto future_result = add_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_predicate_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::removePredicate(const plansys2::Predicate & predicate)
{
  return removePredicateNode(predicate);
}

bool
ProblemExpertClient::removePredicateNode(const plansys2_msgs::msg::Node & predicate)
{
  while (!remove_problem_predicate_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_predicate_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = predicate;

  auto future_result = remove_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_predicate_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::existPredicate(const plansys2::Predicate & predicate)
{
  return existPredicateNode(predicate);
}

bool
ProblemExpertClient::existPredicateNode(const plansys2_msgs::msg::Node & predicate)
{
  while (!exist_problem_predicate_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      exist_problem_predicate_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  request->node = predicate;

  auto future_result = exist_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  return future_result.get()->exist;
}

std::optional<plansys2::Predicate>
ProblemExpertClient::getPredicate(const std::string & predicate)
{
  return getPredicateNode(predicate);
}

std::optional<plansys2_msgs::msg::Node>
ProblemExpertClient::getPredicateNode(const std::string & predicate)
{
  while (!get_problem_predicate_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = predicate;

  auto future_result = get_problem_predicate_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->node;
  } else {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

std::vector<plansys2::Function>
ProblemExpertClient::getFunctions()
{
  auto functions = getFunctionNodes();
  std::vector<plansys2::Function> ret;
  ret.reserve(functions.size());
  std::transform(
    functions.begin(), functions.end(), std::back_inserter(ret),
    [](plansys2_msgs::msg::Node item)
    {
      return plansys2::Function(item);
    });
  return ret;
}

std::vector<plansys2_msgs::msg::Node>
ProblemExpertClient::getFunctionNodes()
{
  while (!get_problem_functions_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto future_result = get_problem_functions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->states;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addFunction(const plansys2::Function & function)
{
  return addFunctionNode(function);
}

bool
ProblemExpertClient::addFunctionNode(const plansys2_msgs::msg::Node & function)
{
  while (!add_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  auto future_result = add_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
      node_,
      future_result,
      std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_function_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::removeFunction(const plansys2::Function & function)
{
  return removeFunctionNode(function);
}

bool
ProblemExpertClient::removeFunctionNode(const plansys2_msgs::msg::Node & function)
{
  while (!remove_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  auto future_result = remove_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_function_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::existFunction(const plansys2::Function & function)
{
  return existFunctionNode(function);
}

bool
ProblemExpertClient::existFunctionNode(const plansys2_msgs::msg::Node & function)
{
  while (!exist_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      exist_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  request->node = function;

  auto future_result = exist_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  return future_result.get()->exist;
}

bool ProblemExpertClient::updateFunction(const plansys2::Function & function)
{
  return updateFunctionNode(function);
}

bool ProblemExpertClient::updateFunctionNode(const plansys2_msgs::msg::Node & function)
{
  while (!update_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      update_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  auto future_result = update_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      update_problem_function_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

std::optional<plansys2::Function>
ProblemExpertClient::getFunction(const std::string & function)
{
  return getFunctionNode(function);
}

std::optional<plansys2_msgs::msg::Node>
ProblemExpertClient::getFunctionNode(const std::string & function)
{
  while (!get_problem_function_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = function;

  auto future_result = get_problem_function_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->node;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

plansys2::Goal
ProblemExpertClient::getGoal()
{
  return getGoalTree();
}

plansys2_msgs::msg::Tree
ProblemExpertClient::getGoalTree()
{
  plansys2_msgs::msg::Tree ret;

  while (!get_problem_goal_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_goal_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemGoal::Request>();

  auto future_result = get_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    return future_result.get()->tree;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_goal_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::setGoal(const plansys2::Goal & goal)
{
  return setGoalTree(goal);
}

bool
ProblemExpertClient::setGoalTree(const plansys2_msgs::msg::Tree & goal)
{
  while (!add_problem_goal_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_goal_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemGoal::Request>();
  request->tree = goal;

  auto future_result = add_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_goal_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::isGoalSatisfied(const plansys2::Goal & goal)
{
  return isGoalTreeSatisfied(goal);
}

bool
ProblemExpertClient::isGoalTreeSatisfied(const plansys2_msgs::msg::Tree & goal)
{
  while (!is_problem_goal_satisfied_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      is_problem_goal_satisfied_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::IsProblemGoalSatisfied::Request>();
  request->tree = goal;

  auto future_result = is_problem_goal_satisfied_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return future_result.get()->satisfied;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      is_problem_goal_satisfied_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::clearGoal()
{
  while (!remove_problem_goal_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_goal_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemGoal::Request>();

  auto future_result = remove_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_goal_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

bool
ProblemExpertClient::clearKnowledge()
{
  while (!clear_problem_knowledge_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      clear_problem_knowledge_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::ClearProblemKnowledge::Request>();

  auto future_result = clear_problem_knowledge_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      clear_problem_knowledge_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return false;
  }
}

std::string
ProblemExpertClient::getProblem()
{
  while (!get_problem_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblem::Request>();

  auto future_result = get_problem_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    return future_result.get()->problem;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

}  // namespace plansys2
