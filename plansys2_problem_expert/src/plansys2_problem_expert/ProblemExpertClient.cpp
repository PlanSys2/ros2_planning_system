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

namespace plansys2
{

ProblemExpertClient::ProblemExpertClient(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node)
{
  add_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::AddProblemGoal>(
    "problem_expert/add_problem_goal");
  add_problem_instance_client_ = node_->create_client<plansys2_msgs::srv::AddProblemInstance>(
    "problem_expert/add_problem_instance");
  add_problem_predicate_client_ = node_->create_client<plansys2_msgs::srv::AddProblemPredicate>(
    "problem_expert/add_problem_predicate");
  add_problem_function_client_ = node_->create_client<plansys2_msgs::srv::AddProblemFunction>(
    "problem_expert/add_problem_function");
  get_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::GetProblemGoal>(
    "problem_expert/get_problem_goal");
  get_problem_instance_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "problem_expert/get_problem_instance");
  get_problem_instances_client_ = node_->create_client<plansys2_msgs::srv::GetProblemInstances>(
    "problem_expert/get_problem_instances");
  get_problem_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemPredicateDetails>(
    "problem_expert/get_problem_predicate");
  get_problem_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetProblemPredicates>(
    "problem_expert/get_problem_predicates");
  get_problem_function_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemFunctionDetails>(
    "problem_expert/get_problem_function");
  get_problem_functions_client_ = node_->create_client<plansys2_msgs::srv::GetProblemFunctions>(
    "problem_expert/get_problem_functions");
  get_problem_client_ = node_->create_client<plansys2_msgs::srv::GetProblem>(
    "problem_expert/get_problem");
  remove_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::RemoveProblemGoal>(
    "problem_expert/remove_problem_goal");
  clear_problem_knowledge_client_ = node_->create_client<plansys2_msgs::srv::ClearProblemKnowledge>(
    "problem_expert/clear_problem_knowledge");
  remove_problem_instance_client_ =
    node_->create_client<plansys2_msgs::srv::RemoveProblemInstance>(
    "problem_expert/remove_problem_instance");
  remove_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::RemoveProblemPredicate>(
    "problem_expert/remove_problem_predicate");
  remove_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::RemoveProblemFunction>(
    "problem_expert/remove_problem_function");
  exist_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::ExistProblemPredicate>(
    "problem_expert/exist_problem_predicate");
  exist_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::ExistProblemFunction>(
    "problem_expert/exist_problem_function");
  update_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::UpdateProblemFunction>(
    "problem_expert/update_problem_function");
  is_problem_goal_satisfied_client_ =
    node_->create_client<plansys2_msgs::srv::IsProblemGoalSatisfied>(
    "problem_expert/is_problem_goal_satisfied");
}

std::vector<parser::pddl::tree::Instance>
ProblemExpertClient::getInstances()
{
  std::vector<parser::pddl::tree::Instance> ret;

  while (!get_problem_instances_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
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
    return ret;
  }

  if (future_result.get()->success) {
    for (size_t i = 0; i < future_result.get()->instances.size(); i++) {
      parser::pddl::tree::Instance instance;
      instance.name = future_result.get()->instances[i];
      instance.type = future_result.get()->types[i];
      ret.push_back(instance);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      node_->get_namespace() <<
        get_problem_instances_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::addInstance(const parser::pddl::tree::Instance & instance)
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

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemInstance::Request>();
  request->instance = instance.name;
  request->type = instance.type;

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
ProblemExpertClient::removeInstance(const std::string & name)
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

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemInstance::Request>();
  request->instance = name;

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

std::optional<parser::pddl::tree::Instance>
ProblemExpertClient::getInstance(const std::string & name)
{
  parser::pddl::tree::Instance ret;
  bool found = false;

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
    return ret;
  }

  if (future_result.get()->success) {
    ret.name = name;
    ret.type = future_result.get()->type;

    return ret;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instance_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

std::vector<parser::pddl::tree::Predicate>
ProblemExpertClient::getPredicates()
{
  std::vector<parser::pddl::tree::Predicate> ret;

  while (!get_problem_predicates_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemPredicates::Request>();

  auto future_result = get_problem_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    for (size_t i = 0; i < future_result.get()->predicates.size(); i++) {
      parser::pddl::tree::Predicate predicate;
      predicate.fromString(future_result.get()->predicates[i]);
      ret.push_back(predicate);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::addPredicate(const parser::pddl::tree::Predicate & predicate)
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

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemPredicate::Request>();
  request->predicate = predicate.name;

  for (const auto & parameter : predicate.parameters) {
    request->arguments.push_back(parameter.name);
  }

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
ProblemExpertClient::removePredicate(const parser::pddl::tree::Predicate & predicate)
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

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemPredicate::Request>();
  request->predicate = predicate.name;

  for (const auto & parameter : predicate.parameters) {
    request->arguments.push_back(parameter.name);
  }

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
ProblemExpertClient::existPredicate(const parser::pddl::tree::Predicate & predicate)
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

  auto request = std::make_shared<plansys2_msgs::srv::ExistProblemPredicate::Request>();
  request->predicate = predicate.name;

  for (const auto & parameter : predicate.parameters) {
    request->arguments.push_back(parameter.name);
  }

  auto future_result = exist_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  return future_result.get()->exist;
}

std::optional<parser::pddl::tree::Predicate>
ProblemExpertClient::getPredicate(const std::string & expr)
{
  parser::pddl::tree::Predicate ret;

  while (!get_problem_predicate_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemPredicateDetails::Request>();

  request->predicate = expr;

  auto future_result = get_problem_predicate_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    ret.name = future_result.get()->name;

    for (size_t i = 0; i < future_result.get()->param_names.size(); ++i) {
      parser::pddl::tree::Param param;
      param.name = future_result.get()->param_names[i];
      param.type = future_result.get()->param_types[i];
      ret.parameters.push_back(param);
    }

    return ret;
  } else {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

std::vector<parser::pddl::tree::Function>
ProblemExpertClient::getFunctions()
{
  std::vector<parser::pddl::tree::Function> ret;

  while (!get_problem_functions_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemFunctions::Request>();

  auto future_result = get_problem_functions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    for (size_t i = 0; i < future_result.get()->functions.size(); i++) {
      parser::pddl::tree::Function function;
      function.fromString(future_result.get()->functions[i]);
      ret.push_back(function);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::addFunction(const parser::pddl::tree::Function & function)
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

  auto request =
    std::make_shared<plansys2_msgs::srv::AddProblemFunction::Request>();
  request->function = function.name;

  for (const auto & parameter : function.parameters) {
    request->arguments.push_back(parameter.name);
  }

  request->value = function.value;

  auto future_result =
    add_problem_function_client_->async_send_request(request);

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
ProblemExpertClient::removeFunction(const parser::pddl::tree::Function & function)
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

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemFunction::Request>();
  request->function = function.name;

  for (const auto & parameter : function.parameters) {
    request->arguments.push_back(parameter.name);
  }

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
ProblemExpertClient::existFunction(const parser::pddl::tree::Function & function)
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

  auto request = std::make_shared<plansys2_msgs::srv::ExistProblemFunction::Request>();
  request->function = function.name;

  for (const auto & parameter : function.parameters) {
    request->arguments.push_back(parameter.name);
  }

  auto future_result = exist_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  return future_result.get()->exist;
}

bool ProblemExpertClient::updateFunction(const parser::pddl::tree::Function & function)
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

  auto request = std::make_shared<plansys2_msgs::srv::UpdateProblemFunction::Request>();
  request->function = function.name;

  for (const auto & parameter : function.parameters) {
    request->arguments.push_back(parameter.name);
  }

  request->value = function.value;

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

std::optional<parser::pddl::tree::Function>
ProblemExpertClient::getFunction(const std::string & expr)
{
  parser::pddl::tree::Function ret;

  while (!get_problem_function_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemFunctionDetails::Request>();

  request->function = expr;

  auto future_result = get_problem_function_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    ret.name = future_result.get()->name;

    for (size_t i = 0; i < future_result.get()->param_names.size(); ++i) {
      parser::pddl::tree::Param param;
      param.name = future_result.get()->param_names[i];
      param.type = future_result.get()->param_types[i];
      ret.parameters.push_back(param);
    }

    ret.value = future_result.get()->value;

    return ret;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
}

parser::pddl::tree::Goal
ProblemExpertClient::getGoal()
{
  parser::pddl::tree::Goal ret;

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
    ret.fromString(future_result.get()->goal);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_goal_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::setGoal(const parser::pddl::tree::Goal & goal)
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
  request->goal = goal.toString();

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

bool
ProblemExpertClient::isGoalSatisfied(const parser::pddl::tree::Goal & goal)
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
  request->goal = goal.toString();
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


std::string
ProblemExpertClient::getProblem()
{
  std::string ret;
  while (!get_problem_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
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
    return ret;
  }

  if (future_result.get()->success) {
    ret = future_result.get()->problem;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
  }

  return ret;
}

}  // namespace plansys2
