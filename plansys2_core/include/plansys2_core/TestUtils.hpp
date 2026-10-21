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

#ifndef PLANSYS2_CORE__TESTUTILS_HPP_
#define PLANSYS2_CORE__TESTUTILS_HPP_

#include <atomic>
#include <memory>
#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

namespace plansys2
{
namespace testing
{

/**
 * @brief Spins an executor in a thread of its own while it exists.
 *
 * The thread stops and is joined when this object is destroyed, also when a failed ASSERT
 * returns early from a test: a joinable thread left behind aborts the whole test program.
 */
class ExecutorThread
{
public:
  explicit ExecutorThread(rclcpp::Executor & executor)
  : executor_(executor),
    thread_([this]() {
        // spin(), not spin_some(): a MultiThreadedExecutor uses all its threads only there
        while (!stop_ && rclcpp::ok()) {
          executor_.spin();
        }
        finished_ = true;
      })
  {
  }

  ~ExecutorThread()
  {
    stop_ = true;
    // Again until it ends: a cancel before spin() starts is lost
    while (!finished_) {
      executor_.cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ExecutorThread(const ExecutorThread &) = delete;
  ExecutorThread & operator=(const ExecutorThread &) = delete;

private:
  rclcpp::Executor & executor_;
  std::atomic<bool> stop_ {false};
  std::atomic<bool> finished_ {false};
  std::thread thread_;
};

/**
 * @brief Spins a node in a thread of its own while it exists: the node of some clients in a
 * test, which get their answers from its executor (multi-threaded, as in plansys2_node).
 */
class NodeSpinner
{
public:
  template<typename NodeT>
  explicit NodeSpinner(const std::shared_ptr<NodeT> & node)
  {
    executor_.add_node(node->get_node_base_interface());
    thread_ = std::make_unique<ExecutorThread>(executor_);
  }

  ~NodeSpinner() {thread_.reset();}

  NodeSpinner(const NodeSpinner &) = delete;
  NodeSpinner & operator=(const NodeSpinner &) = delete;

private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<ExecutorThread> thread_;
};

/**
 * @brief The lifecycle state of a fake component, active: PlanSys2 clients wait for it.
 *
 * @param[in] node The node of the fake.
 * @param[in] component Its name, as the clients call it (problem_expert, executor...).
 */
template<typename NodeT>
rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr
active_state(NodeT & node, const std::string & component)
{
  return node.template create_service<lifecycle_msgs::srv::GetState>(
    component + "/get_state",
    [](const lifecycle_msgs::srv::GetState::Request::SharedPtr,
    lifecycle_msgs::srv::GetState::Response::SharedPtr response) {
      response->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
}

/**
 * @brief A plain service call, as any ROS node makes it: what a server answers in any state.
 *
 * PlanSys2 clients wait until their component is active; this one does not.
 * @return The response, or nullptr without server or answer within timeout.
 */
template<typename ServiceT>
typename ServiceT::Response::SharedPtr call(
  const rclcpp::Node::SharedPtr & spun_node, const std::string & name,
  typename ServiceT::Request::SharedPtr request,
  std::chrono::seconds timeout = std::chrono::seconds(5))
{
  auto client = spun_node->create_client<ServiceT>(name);
  if (!client->wait_for_service(timeout)) {
    return nullptr;
  }
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(future);
    return nullptr;
  }
  return future.get();
}

/**
 * @brief Wait, without spinning, until a condition holds.
 *
 * @param[in] condition What to wait for; something else spins the nodes it depends on.
 * @param[in] timeout How long to wait at most.
 * @return Whether the condition holds.
 */
template<typename Condition>
bool wait_for(
  Condition condition, std::chrono::nanoseconds timeout = std::chrono::seconds(10))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() >= end || !rclcpp::ok()) {
      return condition();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

// Whether a server of the service is known: Humble's nodes have no count_services
template<typename NodeT>
bool has_service(NodeT & node, const std::string & name)
{
#if RCLCPP_VERSION_MAJOR >= 21
  return node.count_services(name) > 0;
#else
  return node.get_service_names_and_types().count("/" + name) > 0;
#endif
}

}  // namespace testing
}  // namespace plansys2

#endif  // PLANSYS2_CORE__TESTUTILS_HPP_
