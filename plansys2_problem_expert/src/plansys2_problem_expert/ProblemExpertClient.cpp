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
    "/problem_expert/add_problem_goal");
  add_problem_instance_client_ = node_->create_client<plansys2_msgs::srv::AddProblemInstance>(
    "/problem_expert/add_problem_instance");
  add_problem_predicate_client_ = node_->create_client<plansys2_msgs::srv::AddProblemPredicate>(
    "/problem_expert/add_problem_predicate");
  get_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::GetProblemGoal>(
    "/problem_expert/get_problem_goal");
  get_problem_instance_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "/problem_expert/get_problem_instance");
  get_problem_instances_client_ = node_->create_client<plansys2_msgs::srv::GetProblemInstances>(
    "/problem_expert/get_problem_instances");
  get_problem_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemPredicateDetails>(
    "/problem_expert/get_problem_predicate");
  get_problem_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetProblemPredicates>(
    "/problem_expert/get_problem_predicates");
  get_problem_client_ = node_->create_client<plansys2_msgs::srv::GetProblem>(
    "/problem_expert/get_problem");
  remove_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::RemoveProblemGoal>(
    "/problem_expert/remove_problem_goal");
  remove_problem_instance_client_ =
    node_->create_client<plansys2_msgs::srv::RemoveProblemInstance>(
    "/problem_expert/remove_problem_instance");
  remove_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::RemoveProblemPredicate>(
    "/problem_expert/remove_problem_predicate");
}

std::vector<Instance>
ProblemExpertClient::getInstances()
{
  std::vector<Instance> ret;

  while (!get_problem_instances_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/get_problem_instances service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstances::Request>();

  auto future_result = get_problem_instances_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    for (size_t i = 0; i < future_result.get()->instances.size(); i++) {
      Instance instance;
      instance.name = future_result.get()->instances[i];
      instance.type = future_result.get()->types[i];
      ret.push_back(instance);
    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/get_problem_instances: %s",
      future_result.get()->error_info.c_str());
  }

  return ret;
}

bool
ProblemExpertClient::addInstance(const Instance & instance)
{
  while (!add_problem_instance_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/add_problem_instance service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemInstance::Request>();
  request->instance = instance.name;
  request->type = instance.type;

  auto future_result = add_problem_instance_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/add_problem_instance: %s",
      future_result.get()->error_info.c_str());
    return false;
  }
}

bool
ProblemExpertClient::removeInstance(const std::string & name)
{
  while (!remove_problem_instance_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/remove_problem_instance service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemInstance::Request>();
  request->instance = name;

  auto future_result = remove_problem_instance_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/remove_problem_instance: %s",
      future_result.get()->error_info.c_str());
    return false;
  }
}

std::optional<plansys2::Instance>
ProblemExpertClient::getInstance(const std::string & name)
{
  plansys2::Instance ret;
  bool found = false;

  while (!get_problem_instance_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/get_problem_instance_details client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstanceDetails::Request>();

  request->instance = name;

  auto future_result = get_problem_instance_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    ret.name = name;
    ret.type = future_result.get()->type;

    return ret;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "error calling /problem_expert/get_problem_instance_details: %s",
      future_result.get()->error_info.c_str());
    return {};
  }
}

std::vector<Predicate>
ProblemExpertClient::getPredicates()
{
  std::vector<Predicate> ret;

  while (!get_problem_predicates_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/get_problem_predicates service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemPredicates::Request>();

  auto future_result = get_problem_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    for (size_t i = 0; i < future_result.get()->predicates.size(); i++) {
      Predicate predicate;
      predicate.fromString(future_result.get()->predicates[i]);
      ret.push_back(predicate);
    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/get_problem_predicates: %s",
      future_result.get()->error_info.c_str());
  }

  return ret;
}

bool
ProblemExpertClient::addPredicate(const Predicate & predicate)
{
  while (!add_problem_predicate_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/add_problem_predicate service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemPredicate::Request>();
  request->predicate = predicate.name;

  for (const auto & parameter : predicate.parameters) {
    request->arguments.push_back(parameter.name);
  }

  auto future_result = add_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/add_problem_predicate: %s",
      future_result.get()->error_info.c_str());
    return false;
  }
}

bool
ProblemExpertClient::removePredicate(const Predicate & predicate)
{
  while (!remove_problem_predicate_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/remove_problem_predicate service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemPredicate::Request>();
  request->predicate = predicate.name;

  for (const auto & parameter : predicate.parameters) {
    request->arguments.push_back(parameter.name);
  }

  auto future_result = remove_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/remove_problem_predicate: %s",
      future_result.get()->error_info.c_str());
    return false;
  }
}

Goal
ProblemExpertClient::getGoal()
{
  Goal ret;

  while (!get_problem_goal_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/get_problem_goal service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemGoal::Request>();

  auto future_result = get_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    ret.fromString(future_result.get()->goal);
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/get_problem_goal: %s",
      future_result.get()->error_info.c_str());
  }

  return ret;
}

bool
ProblemExpertClient::setGoal(const Goal & goal)
{
  while (!add_problem_goal_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/add_problem_goal service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemGoal::Request>();
  request->goal = goal.toString();

  auto future_result = add_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/add_problem_goal: %s",
      future_result.get()->error_info.c_str());
    return false;
  }
}


bool
ProblemExpertClient::clearGoal()
{
  while (!remove_problem_goal_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/remove_problem_goal service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemGoal::Request>();

  auto future_result = remove_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  if (future_result.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/remove_problem_goal: %s",
      future_result.get()->error_info.c_str());
    return false;
  }
}


std::string
ProblemExpertClient::getProblem()
{
  std::string ret;
  while (!get_problem_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/problem_expert/get_problem service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblem::Request>();

  auto future_result = get_problem_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    ret = future_result.get()->problem;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "error calling /problem_expert/get_problem: %s",
      future_result.get()->error_info.c_str());
  }

  return ret;
}

}  // namespace plansys2
