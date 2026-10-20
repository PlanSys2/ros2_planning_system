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
#include <stdexcept>
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
  std::shared_ptr<ClientNode> node, const std::string & managed_node)
: node_(node), managed_node_(managed_node)
{
  client_get_state_ = std::make_shared<ServiceClient<lifecycle_msgs::srv::GetState>>(
    node_, managed_node_ + "/get_state");
  client_change_state_ = std::make_shared<ServiceClient<lifecycle_msgs::srv::ChangeState>>(
    node_, managed_node_ + "/change_state");
}

unsigned int
LifecycleServiceClient::get_state(std::chrono::seconds time_out)
{
  // One attempt: the startup asks again, and a call without answer leaves a new client
  auto state = client_get_state_->call(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>(), time_out, 1, time_out);
  if (!state) {
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  RCLCPP_INFO(
    get_logger(), "Node %s has current state %s.",
    managed_node_.c_str(), state->current_state.label.c_str());
  return state->current_state.id;
}

bool
LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out)
{
  // One attempt: a transition is not repeated (the startup checks the state it reached)
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = client_change_state_->call(request, time_out, 1, time_out);
  if (!result) {
    return false;
  }
  if (result->success) {
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

// Triggers a transition and waits for its state. A lost response (DDS may drop it while the
// nodes are still discovering each other) is not a failure if the node got there anyway
bool
transition_to(
  LifecycleServiceClient & client, const std::string & name, std::uint8_t transition,
  unsigned int expected, std::chrono::seconds timeout)
{
  if (!client.change_state(transition, timeout)) {
    if (!rclcpp::ok() || !wait_for_state(client, name, expected, timeout)) {
      return false;
    }
    RCLCPP_WARN(
      client.get_logger(), "No answer to the transition of %s, but it made it: going on",
      name.c_str());
    return true;
  }
  return wait_for_state(client, name, expected, timeout);
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
      !transition_to(
        client, name, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE, timeout))
    {
      return false;
    }
  }

  for (const auto & name : activate_order) {
    auto & client = *manager_nodes.at(name);
    if (!rclcpp::ok() ||
      !transition_to(
        client, name, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE, timeout))
    {
      return false;
    }
  }

  return true;
}

}  // namespace plansys2
