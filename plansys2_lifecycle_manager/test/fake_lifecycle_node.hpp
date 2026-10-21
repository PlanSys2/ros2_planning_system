// Copyright 2026 Intelligent Robotics Lab
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

#ifndef FAKE_LIFECYCLE_NODE_HPP_
#define FAKE_LIFECYCLE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "plansys2_core/TestUtils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Plain node that offers the lifecycle services of `name`, answering as told
class FakeLifecycleNode : public rclcpp::Node
{
public:
  explicit FakeLifecycleNode(const std::string & name)
  : Node(name + "_fake")
  {
    get_state_ = create_service<lifecycle_msgs::srv::GetState>(
      name + "/get_state",
      [this](
        const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
        std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
        get_state_calls++;
        std::this_thread::sleep_for(get_state_delay.load());
        response->current_state.id = state;
      });
    change_state_ = create_service<lifecycle_msgs::srv::ChangeState>(
      name + "/change_state",
      [this](
        std::shared_ptr<rclcpp::Service<lifecycle_msgs::srv::ChangeState>> service,
        std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>) {
        std::this_thread::sleep_for(change_state_delay.load());
        if (on_change_state) {
          on_change_state();
        }
        // As DDS may drop it while the nodes are still discovering each other
        if (lose_change_state_response) {
          return;
        }
        lifecycle_msgs::srv::ChangeState::Response response;
        response.success = change_state_success;
        service->send_response(*header, response);
      });
  }

  std::atomic<uint8_t> state {lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED};
  std::atomic<bool> change_state_success {true};
  std::atomic<bool> lose_change_state_response {false};
  std::atomic<std::chrono::milliseconds> get_state_delay {std::chrono::milliseconds(0)};
  std::atomic<std::chrono::milliseconds> change_state_delay {std::chrono::milliseconds(0)};
  std::function<void()> on_change_state;
  std::atomic<int> get_state_calls {0};

private:
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_;
  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_;
};

// Lifecycle node whose configure or activate transition can fail
class FailingLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FailingLifecycleNode(const std::string & name, bool fail_configure, bool fail_activate)
  : LifecycleNode(name), fail_configure_(fail_configure), fail_activate_(fail_activate) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return fail_configure_ ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return fail_activate_ ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

private:
  bool fail_configure_;
  bool fail_activate_;
};

// Spins the nodes added to it in its own thread from start() until it is destroyed
class Spinner
{
public:
  // Keeps the node alive until the executor stops
  template<typename NodeT>
  void add(std::shared_ptr<NodeT> node)
  {
    keep_alive_.push_back(node);
    exe_.add_node(node->get_node_base_interface());
  }

  void start() {thread_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);}

private:
  std::vector<std::shared_ptr<void>> keep_alive_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> thread_;
};

#endif  // FAKE_LIFECYCLE_NODE_HPP_
