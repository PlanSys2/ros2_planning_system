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
#include <thread>
#include <vector>

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
  if (!client_get_state_) {
    RCLCPP_ERROR(get_logger(), "init() was not called");
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
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
  // Never get() before it is ready: it would block forever if the node does not answer
  auto future_status = wait_for_result(future_result, time_out);
  if (future_status != std::future_status::ready) {
    client_get_state_->remove_pending_request(future_result);
    RCLCPP_ERROR(
      get_logger(), "Server time out while getting current state for node %s",
      managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  auto state = future_result.get();
  // We have an successful answer. So let's print the current state.
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
  if (!client_change_state_) {
    RCLCPP_ERROR(get_logger(), "init() was not called");
    return false;
  }
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
    client_change_state_->remove_pending_request(future_result);
    RCLCPP_ERROR(
      get_logger(), "Server time out while changing the state of node %s",
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
      get_logger(), "Failed to trigger transition %u in node %s",
      static_cast<unsigned int>(transition), managed_node_.c_str());
    return false;
  }
}

namespace
{

// Polls the state until it is the expected one, the time is up or ROS shuts down
bool
wait_for_state(
  LifecycleServiceClient & client, const std::string & name, unsigned int expected,
  std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok()) {
    if (client.get_state(timeout) == expected) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_ERROR(
    client.get_logger(), "Node %s did not reach the state %u", name.c_str(), expected);
  return false;
}

}  // namespace

bool
startup_function(
  const std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout)
{
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;

  // Configured in this order (the domain expert may validate with the planner), then activated
  const std::vector<std::string> configure_order =
  {"planner", "domain_expert", "problem_expert", "executor"};
  const std::vector<std::string> activate_order =
  {"domain_expert", "problem_expert", "planner", "executor"};

  for (const auto & name : configure_order) {
    auto it = manager_nodes.find(name);
    if (it == manager_nodes.end() || it->second == nullptr) {
      RCLCPP_ERROR(
        rclcpp::get_logger("plansys2_lifecycle_manager"),
        "No lifecycle client for node %s", name.c_str());
      return false;
    }
  }

  for (const auto & name : configure_order) {
    auto & client = *manager_nodes.at(name);
    if (!rclcpp::ok() ||
      !client.change_state(Transition::TRANSITION_CONFIGURE, timeout) ||
      !wait_for_state(client, name, State::PRIMARY_STATE_INACTIVE, timeout))
    {
      return false;
    }
  }

  for (const auto & name : activate_order) {
    auto & client = *manager_nodes.at(name);
    if (!rclcpp::ok() ||
      !client.change_state(Transition::TRANSITION_ACTIVATE, timeout) ||
      !wait_for_state(client, name, State::PRIMARY_STATE_ACTIVE, timeout))
    {
      return false;
    }
  }

  return true;
}

}  // namespace plansys2
