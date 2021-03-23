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
  get_domain_client_ = node_->create_client<plansys2_msgs::srv::GetDomain>(
    "domain_expert/get_domain");
  get_types_client_ = node_->create_client<plansys2_msgs::srv::GetDomainTypes>(
    "domain_expert/get_domain_types");
  get_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetDomainPredicates>(
    "domain_expert/get_domain_predicates");
  get_functions_client_ = node_->create_client<plansys2_msgs::srv::GetDomainFunctions>(
    "domain_expert/get_domain_functions");
  get_actions_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_actions");
  get_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetDomainPredicateDetails>(
    "domain_expert/get_domain_predicate_details");
  get_function_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetDomainFunctionDetails>(
    "domain_expert/get_domain_function_details");
  get_action_details_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActionDetails>(
    "domain_expert/get_domain_action_details");
}

std::vector<std::string>
DomainExpertClient::getTypes()
{
  std::vector<std::string> ret;

  while (!get_types_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_types_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainTypes::Request>();

  auto future_result = get_types_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
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
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_predicates_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainPredicates::Request>();

  auto future_result = get_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  ret = future_result.get()->predicates;

  return ret;
}

std::optional<parser::pddl::tree::Predicate>
DomainExpertClient::getPredicate(const std::string & predicate)
{
  parser::pddl::tree::Predicate ret;
  bool found = false;

  while (!get_predicate_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_predicate_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainPredicateDetails::Request>();

  request->predicate = predicate;

  auto future_result = get_predicate_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    parser::pddl::tree::Predicate ret;
    ret.name = future_result.get()->name;

    for (size_t i = 0; i < future_result.get()->param_names.size(); i++) {
      parser::pddl::tree::Param param;
      param.name = future_result.get()->param_names[i];
      param.type = future_result.get()->param_types[i];
      ret.parameters.push_back(param);
    }

    return ret;

  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_predicate_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
  return {};
}

std::vector<std::string>
DomainExpertClient::getFunctions()
{
  std::vector<std::string> ret;

  while (!get_functions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_functions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainFunctions::Request>();

  auto future_result = get_functions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  ret = future_result.get()->functions;

  return ret;
}

std::optional<parser::pddl::tree::Function>
DomainExpertClient::getFunction(const std::string & function)
{
  parser::pddl::tree::Function ret;
  bool found = false;

  while (!get_function_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_function_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainFunctionDetails::Request>();

  request->function = function;

  auto future_result = get_function_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    parser::pddl::tree::Function ret;
    ret.name = future_result.get()->name;

    for (size_t i = 0; i < future_result.get()->param_names.size(); i++) {
      parser::pddl::tree::Param param;
      param.name = future_result.get()->param_names[i];
      param.type = future_result.get()->param_types[i];
      ret.parameters.push_back(param);
    }

    return ret;

  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_function_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
  return {};
}

std::vector<std::string>
DomainExpertClient::getActions()
{
  std::vector<std::string> ret;

  while (!get_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_actions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto future_result = get_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  for (size_t i = 0; i < future_result.get()->actions.size(); i++) {
    if (future_result.get()->type[i] == "action") {
      ret.push_back(future_result.get()->actions[i]);
    }
  }

  return ret;
}

std::optional<parser::pddl::tree::Action>
DomainExpertClient::getAction(const std::string & action)
{
  parser::pddl::tree::Action ret;
  bool found = false;

  while (!get_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActionDetails::Request>();

  request->action = action;

  auto future_result = get_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    if (future_result.get()->type == "action") {
      parser::pddl::tree::Action ret;
      ret.name = future_result.get()->name;

      for (size_t i = 0; i < future_result.get()->param_names.size(); i++) {
        parser::pddl::tree::Param param;
        param.name = future_result.get()->param_names[i];
        param.type = future_result.get()->param_types[i];
        ret.parameters.push_back(param);
      }

      ret.preconditions.fromString(future_result.get()->at_start_requirements);
      ret.effects.fromString(future_result.get()->at_start_effects);

      return ret;
    } else {
      return {};
    }
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
  return {};
}

std::vector<std::string>
DomainExpertClient::getDurativeActions()
{
  std::vector<std::string> ret;

  while (!get_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_actions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto future_result = get_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  for (size_t i = 0; i < future_result.get()->actions.size(); i++) {
    if (future_result.get()->type[i] == "durative-action") {
      ret.push_back(future_result.get()->actions[i]);
    }
  }

  return ret;
}

std::optional<parser::pddl::tree::DurativeAction>
DomainExpertClient::getDurativeAction(const std::string & action)
{
  parser::pddl::tree::DurativeAction ret;
  bool found = false;

  while (!get_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActionDetails::Request>();

  request->action = action;

  auto future_result = get_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  if (future_result.get()->success) {
    if (future_result.get()->type == "durative-action") {
      ret.name = future_result.get()->name;

      for (size_t i = 0; i < future_result.get()->param_names.size(); i++) {
        parser::pddl::tree::Param param;
        param.name = future_result.get()->param_names[i];
        param.type = future_result.get()->param_types[i];
        ret.parameters.push_back(param);
      }

      ret.at_start_requirements.fromString(future_result.get()->at_start_requirements);
      ret.over_all_requirements.fromString(future_result.get()->over_all_requirements);
      ret.at_end_requirements.fromString(future_result.get()->at_end_requirements);
      ret.at_start_effects.fromString(future_result.get()->at_start_effects);
      ret.at_end_effects.fromString(future_result.get()->at_end_effects);

      return ret;
    } else {
      return {};
    }
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() << ": " <<
        future_result.get()->error_info);
    return {};
  }
  return {};
}

std::string
DomainExpertClient::getDomain()
{
  std::string ret;

  while (!get_domain_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_domain_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomain::Request>();

  auto future_result = get_domain_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  ret = future_result.get()->domain;

  return ret;
}
}  // namespace plansys2
