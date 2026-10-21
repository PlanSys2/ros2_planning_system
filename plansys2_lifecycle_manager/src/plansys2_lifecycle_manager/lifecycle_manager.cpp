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
  // Debug: the supervisor asks it all the time
  RCLCPP_DEBUG(
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

Supervisor::Supervisor(
  const std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  const Options & options)
: nodes_(manager_nodes), options_(options)
{
  const auto now = std::chrono::steady_clock::now();
  for (const auto & [name, client] : nodes_) {
    (void)client;
    watch_[name].last_seen = now;
  }
}

void
Supervisor::bring_up(const std::string & name)
{
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;
  auto & client = *nodes_.at(name);
  RCLCPP_WARN(client.get_logger(), "%s started again: configuring and activating it", name.c_str());
  if (!transition_to(
      client, name, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE,
      options_.transition_timeout))
  {
    RCLCPP_ERROR(client.get_logger(), "%s could not be configured", name.c_str());
    return;
  }
  // The executor waits, while it is deactivated for a component that is gone
  if (name == "executor" && degraded_) {
    return;
  }
  if (!transition_to(
      client, name, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE,
      options_.transition_timeout))
  {
    RCLCPP_ERROR(client.get_logger(), "%s could not be activated", name.c_str());
  }
}

void
Supervisor::step()
{
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;

  // In the order of the startup: the domain expert may validate with the planner
  const std::vector<std::string> order = {"planner", "domain_expert", "problem_expert",
    "executor"};
  for (const auto & name : order) {
    auto it = nodes_.find(name);
    if (it == nodes_.end() || !it->second || !rclcpp::ok()) {
      continue;
    }
    auto & watch = watch_[name];
    const auto state = it->second->get_state(std::chrono::seconds(1));
    const auto now = std::chrono::steady_clock::now();
    watch.state = state;
    if (state == State::PRIMARY_STATE_UNKNOWN) {
      if (!watch.lost && now - watch.last_seen > options_.lost_after) {
        watch.lost = true;
        RCLCPP_ERROR(it->second->get_logger(), "%s does not answer", name.c_str());
      }
      continue;
    }
    if (watch.lost) {
      RCLCPP_INFO(it->second->get_logger(), "%s answers again", name.c_str());
    }
    watch.last_seen = now;
    watch.lost = false;
    if (state == State::PRIMARY_STATE_UNCONFIGURED) {
      bring_up(name);
      watch.state = it->second->get_state(std::chrono::seconds(1));
    }
  }

  // Gone for longer than the grace: what runs fails instead of waiting for it
  const auto now = std::chrono::steady_clock::now();
  std::string gone;
  for (const auto & [name, watch] : watch_) {
    if (name != "executor" && watch.lost && now - watch.last_seen > options_.grace) {
      gone = name;
    }
  }
  auto executor = nodes_.find("executor");
  if (executor == nodes_.end() || !executor->second) {
    return;
  }
  if (!gone.empty() && !degraded_) {
    RCLCPP_ERROR(
      executor->second->get_logger(),
      "%s is gone: deactivating the executor, the plans in course fail", gone.c_str());
    degraded_ = true;
    if (watch_["executor"].state == State::PRIMARY_STATE_ACTIVE) {
      transition_to(
        *executor->second, "executor", Transition::TRANSITION_DEACTIVATE,
        State::PRIMARY_STATE_INACTIVE, options_.transition_timeout);
    }
    return;
  }

  // All back, and active: the executor too
  if (degraded_) {
    for (const auto & [name, watch] : watch_) {
      if (name != "executor" && watch.state != State::PRIMARY_STATE_ACTIVE) {
        return;
      }
    }
    degraded_ = false;
    RCLCPP_INFO(executor->second->get_logger(), "Every component is back: activating the executor");
    if (watch_["executor"].state == State::PRIMARY_STATE_INACTIVE) {
      transition_to(
        *executor->second, "executor", Transition::TRANSITION_ACTIVATE,
        State::PRIMARY_STATE_ACTIVE, options_.transition_timeout);
    } else if (watch_["executor"].state == State::PRIMARY_STATE_UNCONFIGURED) {
      bring_up("executor");
    }
  }
}

}  // namespace plansys2
