// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>
#include <map>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

namespace plansys2
{

LifecycleServiceClient::LifecycleServiceClient(
  const std::string & node_name, const std::string & managed_node)
: Node(node_name), managed_node_(managed_node)
{}

void
LifecycleServiceClient::init()
{
  std::string get_state_service_name = managed_node_ + "/get_state";
  std::string change_state_service_name = managed_node_ + "/change_state";
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", get_state_service_name.c_str());
  RCLCPP_INFO(
    get_logger(), "Creating client for service [%s]",
    change_state_service_name.c_str());
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(get_state_service_name);
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    change_state_service_name);
}

unsigned int
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  if (!client_get_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      get_logger(),
      "Service %s is not available.",
      client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  // We send the service request for asking the current
  // state of the lc_talker node.
  auto future_result = client_get_state_->async_send_request(request);
  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);
  auto state = future_result.get();

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node %s",
      managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  // We have an succesful answer. So let's print the current state.
  if (state != nullptr) {
    RCLCPP_INFO(
      get_logger(), "Node %s has current state %s.",
      get_name(), state->current_state.label.c_str());
    return state->current_state.id;
  } else {
    RCLCPP_ERROR(
      get_logger(), "Failed to get current state for node %s", managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

bool
LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (!client_change_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      get_logger(),
      "Service %s is not available.",
      client_change_state_->get_service_name());
    return false;
  }
  // We send the request with the transition we want to invoke.
  auto future_result = client_change_state_->async_send_request(request);
  // Let's wait until we have the answer from the node.
  // If the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node %s",
      managed_node_.c_str());
    return false;
  }
  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
      get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(
      get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
}

bool
startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout)
{
  // configure domain_expert
  {
    if (!manager_nodes["domain_expert"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        timeout))
    {
      return false;
    }

    while (manager_nodes["domain_expert"]->get_state() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      std::cerr << "Waiting for inactive state for domain_expert" << std::endl;
    }
  }

  // configure problem_expert
  {
    if (!manager_nodes["problem_expert"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        timeout))
    {
      return false;
    }

    while (manager_nodes["problem_expert"]->get_state() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      std::cerr << "Waiting for inactive state for problem_expert" << std::endl;
    }
  }

  // configure planner
  {
    if (!manager_nodes["planner"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        timeout))
    {
      return false;
    }

    while (manager_nodes["planner"]->get_state() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // configure executor
  {
    if (!manager_nodes["executor"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        timeout))
    {
      return false;
    }

    while (manager_nodes["executor"]->get_state() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // activate
  {
    if (!rclcpp::ok()) {
      return false;
    }
    if (!manager_nodes["domain_expert"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["problem_expert"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["planner"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["executor"]->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        timeout))
    {
      return false;
    }
    if (!manager_nodes["domain_expert"]->get_state()) {
      return false;
    }
    if (!manager_nodes["problem_expert"]->get_state()) {
      return false;
    }
    if (!manager_nodes["planner"]->get_state()) {
      return false;
    }
    if (!manager_nodes["executor"]->get_state()) {
      return false;
    }
  }
  return true;
}

}  // namespace plansys2
