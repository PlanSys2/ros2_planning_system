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

#ifndef PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

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

namespace plansys2
{

/**
 * @brief Waits for a future to be ready or until a timeout occurs.
 *
 * @tparam FutureT Type of the future.
 * @tparam WaitTimeT Type of the wait duration.
 * @param[in,out] future The future to wait for.
 * @param[in] time_to_wait Maximum time to wait for the result.
 * @return std::future_status Status of the future after waiting.
 */
template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

/**
 * @class plansys2::LifecycleServiceClient
 * @brief Client node to manage the lifecycle of another ROS2 node.
 *
 * Provides methods to query and change the state of a managed node
 * using the lifecycle management services.
 */
class LifecycleServiceClient : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new LifecycleServiceClient object.
   *
   * @param[in] node_name Name for this client node.
   * @param[in] managed_node Name of the node to be managed.
   */
  explicit LifecycleServiceClient(const std::string & node_name, const std::string & managed_node);

  /**
   * @brief Initializes the service clients for lifecycle management.
   */
  void init();

  /**
   * @brief Gets the current state of the managed node.
   *
   * @param[in] time_out Timeout for the service call (default: 3 seconds).
   * @return unsigned int The current state as defined in lifecycle_msgs::msg::State.
   */
  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));

  /**
   * @brief Changes the state of the managed node.
   *
   * @param[in] transition The transition ID to trigger.
   * @param[in] time_out Timeout for the service call (default: 3 seconds).
   * @return true if the transition was successful, false otherwise.
   */
  bool change_state(
    std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds(3));

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

  std::string managed_node_;
};

/**
 * @brief Starts up all managed nodes using their LifecycleServiceClient interfaces.
 *
 * @param[in,out] manager_nodes Map of node names to their LifecycleServiceClient instances.
 * @param[in] timeout Timeout for each startup operation.
 * @return true if all nodes started successfully, false otherwise.
 */
bool
startup_function(
  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout);

}  // namespace plansys2

#endif  // PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
