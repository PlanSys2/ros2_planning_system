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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/node.hpp"

namespace plansys2
{

DomainExpertClient::DomainExpertClient(ClientNode::Interfaces node)
: node_(std::make_shared<ClientNode>(node)),
  active_(std::make_shared<ActiveComponent>(node_, "domain_expert"))
{
  namespace srv = plansys2_msgs::srv;
  get_domain_client_ =
    std::make_shared<ServiceClient<srv::GetDomain>>(node_, "domain_expert/get_domain", active_);
  change_domain_client_ = std::make_shared<ServiceClient<srv::ChangeDomain>>(
    node_, "domain_expert/change_domain", active_);
  get_name_client_ = std::make_shared<ServiceClient<srv::GetDomainName>>(
    node_, "domain_expert/get_domain_name", active_);
  get_types_client_ = std::make_shared<ServiceClient<srv::GetDomainTypes>>(
    node_, "domain_expert/get_domain_types", active_);
  get_constants_client_ = std::make_shared<ServiceClient<srv::GetDomainConstants>>(
    node_, "domain_expert/get_domain_constants", active_);
  get_predicates_client_ = std::make_shared<ServiceClient<srv::GetStates>>(
    node_, "domain_expert/get_domain_predicates", active_);
  get_functions_client_ = std::make_shared<ServiceClient<srv::GetStates>>(
    node_, "domain_expert/get_domain_functions", active_);
  get_derived_predicates_client_ =
    std::make_shared<ServiceClient<srv::GetDomainDerivedPredicateDetails>>(
    node_, "domain_expert/get_domain_derived_predicates", active_);
  get_derived_predicate_details_client_ =
    std::make_shared<ServiceClient<srv::GetDomainDerivedPredicateDetails>>(
    node_, "domain_expert/get_domain_derived_predicate_details", active_);
  get_actions_client_ = std::make_shared<ServiceClient<srv::GetDomainActions>>(
    node_, "domain_expert/get_domain_actions", active_);
  get_durative_actions_client_ = std::make_shared<ServiceClient<srv::GetDomainActions>>(
    node_, "domain_expert/get_domain_durative_actions", active_);
  get_predicate_details_client_ = std::make_shared<ServiceClient<srv::GetNodeDetails>>(
    node_, "domain_expert/get_domain_predicate_details", active_);
  get_function_details_client_ = std::make_shared<ServiceClient<srv::GetNodeDetails>>(
    node_, "domain_expert/get_domain_function_details", active_);
  get_action_details_client_ = std::make_shared<ServiceClient<srv::GetDomainActionDetails>>(
    node_, "domain_expert/get_domain_action_details", active_);
  get_processes_client_ = std::make_shared<ServiceClient<srv::GetDomainActions>>(
    node_, "domain_expert/get_domain_processes", active_);
  get_process_details_client_ = std::make_shared<ServiceClient<srv::GetDomainActionDetails>>(
    node_, "domain_expert/get_domain_process_details", active_);
  get_events_client_ = std::make_shared<ServiceClient<srv::GetDomainActions>>(
    node_, "domain_expert/get_domain_events", active_);
  get_event_details_client_ = std::make_shared<ServiceClient<srv::GetDomainActionDetails>>(
    node_, "domain_expert/get_domain_event_details", active_);
  get_durative_action_details_client_ =
    std::make_shared<ServiceClient<srv::GetDomainDurativeActionDetails>>(
    node_, "domain_expert/get_domain_durative_action_details", active_);

  domain_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "domain_expert/domain",
    rclcpp::QoS(100).transient_local(), [this](std_msgs::msg::String::SharedPtr msg) {
      cached_domain_ = msg->data;
    });
}

std::string
DomainExpertClient::getName()
{
  std::string ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainName::Request>();

  auto response = get_name_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  ret = result.name;

  return ret;
}

std::vector<std::string>
DomainExpertClient::getTypes()
{
  std::vector<std::string> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainTypes::Request>();

  auto response = get_types_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  ret = result.types;

  return ret;
}

std::vector<std::string>
DomainExpertClient::getConstants(const std::string & type)
{
  (void)type;
  std::vector<std::string> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainConstants::Request>();
  request->type = type;

  auto response = get_constants_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  ret = result.constants;

  return ret;
}

std::vector<plansys2::Predicate>
DomainExpertClient::getPredicates()
{
  std::vector<plansys2::Predicate> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto response = get_predicates_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  ret = plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(
    result.states);

  return ret;
}

std::optional<plansys2::Predicate>
DomainExpertClient::getPredicate(const std::string & predicate)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = predicate;

  auto response = get_predicate_details_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return result.node;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_predicate_details_client_->name() << ": " <<
        result.error_info);
    return {};
  }
  return {};
}

std::vector<plansys2::Function>
DomainExpertClient::getFunctions()
{
  std::vector<plansys2::Function> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto response = get_functions_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  ret = plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
    result.states);

  return ret;
}

std::optional<plansys2::Function>
DomainExpertClient::getFunction(const std::string & function)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = function;

  auto response = get_function_details_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return result.node;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_function_details_client_->name() << ": " <<
        result.error_info);
    return {};
  }
  return {};
}

std::vector<plansys2_msgs::msg::Derived>
DomainExpertClient::getDerivedPredicates()
{
  std::vector<plansys2_msgs::msg::Derived> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request>();

  auto response = get_derived_predicates_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  return response->predicates;
}

plansys2::DerivedResolutionGraph
DomainExpertClient::getDerivedResolutionGraph()
{
  plansys2::DerivedResolutionGraph ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request>();

  auto response = get_derived_predicates_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  return plansys2::DerivedResolutionGraph(response->predicates);
}

std::vector<plansys2_msgs::msg::Derived>
DomainExpertClient::getDerivedPredicate(
  const std::string & predicate, const std::vector<std::string> & params)
{
  (void)params;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request>();

  request->predicate = predicate;

  auto response = get_derived_predicate_details_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return result.predicates;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_derived_predicate_details_client_->name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<std::string>
DomainExpertClient::get_names(
  ServiceClient<plansys2_msgs::srv::GetDomainActions>::SharedPtr client)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();
  auto response = client->call(request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }
  return response->actions;
}

plansys2_msgs::msg::Action::SharedPtr
DomainExpertClient::get_details(
  ServiceClient<plansys2_msgs::srv::GetDomainActionDetails>::SharedPtr client,
  const std::string & name, const std::vector<std::string> & params)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActionDetails::Request>();
  request->action = name;
  request->parameters = params;
  auto response = client->call(request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }
  auto result = response;
  if (!result->success) {
    return {};
  }
  return std::make_shared<plansys2_msgs::msg::Action>(result->action);
}

std::vector<std::string>
DomainExpertClient::getProcesses()
{
  return get_names(get_processes_client_);
}

plansys2_msgs::msg::Action::SharedPtr
DomainExpertClient::getProcess(const std::string & process, const std::vector<std::string> & params)
{
  return get_details(get_process_details_client_, process, params);
}

std::vector<std::string>
DomainExpertClient::getEvents()
{
  return get_names(get_events_client_);
}

plansys2_msgs::msg::Action::SharedPtr
DomainExpertClient::getEvent(const std::string & event, const std::vector<std::string> & params)
{
  return get_details(get_event_details_client_, event, params);
}

std::vector<std::string>
DomainExpertClient::getActions()
{
  std::vector<std::string> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto response = get_actions_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  for (size_t i = 0; i < result.actions.size(); i++) {
    ret.push_back(result.actions[i]);
  }

  return ret;
}

plansys2_msgs::msg::Action::SharedPtr
DomainExpertClient::getAction(
  const std::string & action,
  const std::vector<std::string> & params)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActionDetails::Request>();

  request->action = action;
  request->parameters = params;

  auto response = get_action_details_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return std::make_shared<plansys2_msgs::msg::Action>(result.action);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<std::string>
DomainExpertClient::getDurativeActions()
{
  std::vector<std::string> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto response = get_durative_actions_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  for (size_t i = 0; i < result.actions.size(); i++) {
    ret.push_back(result.actions[i]);
  }

  return ret;
}

plansys2_msgs::msg::DurativeAction::SharedPtr
DomainExpertClient::getDurativeAction(
  const std::string & action,
  const std::vector<std::string> & params)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDurativeActionDetails::Request>();

  request->durative_action = action;
  request->parameters = params;

  auto response = get_durative_action_details_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return nullptr;
  }

  auto result = *response;

  if (result.success) {
    return std::make_shared<plansys2_msgs::msg::DurativeAction>(
      result.durative_action);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_action_details_client_->name() << ": " <<
        result.error_info);
    return nullptr;
  }
}

std::string
DomainExpertClient::getDomain(bool use_cache)
{
  node_->serve();  // The domain published meanwhile
  if (use_cache && cached_domain_ != "") {
    return cached_domain_;
  } else {
    return getDomain();
  }
}

std::string
DomainExpertClient::getDomain()
{
  std::string ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetDomain::Request>();

  auto response = get_domain_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  ret = result.domain;

  return ret;
}

bool
DomainExpertClient::changeDomain(const std::string & domain)
{
  auto request = std::make_shared<plansys2_msgs::srv::ChangeDomain::Request>();
  request->domain = domain;

  auto response = change_domain_client_->call(request, std::chrono::seconds(15));
  if (!response) {
    return false;
  }

  auto result = *response;

  if (result.success) {
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      change_domain_client_->name() << ": " <<
        result.error_info);
    return false;
  }
}

}  // namespace plansys2
