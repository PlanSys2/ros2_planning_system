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

#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

DomainExpertClient::DomainExpertClient(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node)
{
  get_types_client_ = node_->create_client<plansys2_msgs::srv::GetDomainTypes>(
    "/domain_expert/get_domain_types");
  get_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetDomainPredicates>(
    "/domain_expert/get_domain_predicates");
  get_actions_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActions>(
    "/domain_expert/get_domain_actions");
  get_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetDomainPredicateDetails>(
    "/domain_expert/get_domain_predicate_details");
  get_action_details_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActionDetails>(
    "/domain_expert/get_domain_action_details");
}

std::vector<std::string>
DomainExpertClient::getTypes()
{
  std::vector<std::string> ret;

  while (!get_types_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/domain_expert/get_domain_types service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainTypes::Request>();

  auto future_result = get_types_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  ret = future_result.get()->types;

  return ret;
}

std::vector<std::string>
DomainExpertClient::getPredicates()
{
  std::vector<std::string> ret;

  while (!get_predicates_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/domain_expert/get_domain_predicates service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainPredicates::Request>();

  auto future_result = get_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  ret = future_result.get()->predicates;

  return ret;
}

std::optional<std::vector<std::string>>
DomainExpertClient::getPredicateParams(const std::string & predicate)
{
  std::vector<std::string> ret;
  bool found = false;

  while (!get_predicate_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/domain_expert/get_domain_predicate_details client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainPredicateDetails::Request>();

  request->predicate = predicate;

  auto future_result = get_predicate_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    return future_result.get()->argument_params;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "error calling /domain_expert/get_domain_predicate_details: %s",
      future_result.get()->error_info.c_str());
    return {};
  }
}

std::vector<std::string>
DomainExpertClient::getActions()
{
  std::vector<std::string> ret;

  while (!get_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/domain_expert/get_domain_actions service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto future_result = get_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  ret = future_result.get()->actions;

  return ret;
}

std::optional<std::vector<std::string>>
DomainExpertClient::getActionParams(const std::string & action)
{
  std::vector<std::string> ret;
  bool found = false;

  while (!get_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR(
      node_->get_logger(),
      "/domain_expert/get_domain_action_details client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActionDetails::Request>();

  request->action = action;

  auto future_result = get_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  if (future_result.get()->success) {
    return future_result.get()->argument_params;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "error calling /domain_expert/get_domain_action_details: %s",
      future_result.get()->error_info.c_str());
    return {};
  }
}

}  // namespace plansys2
