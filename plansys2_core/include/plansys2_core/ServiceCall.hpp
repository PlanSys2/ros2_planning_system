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

#ifndef PLANSYS2_CORE__SERVICECALL_HPP_
#define PLANSYS2_CORE__SERVICECALL_HPP_

#include <chrono>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @brief Waits for the server of a client: wait is its wait_for_service or wait_for_action_server.
 *
 * rclcpp shutting down during the wait throws (NodeNotFoundError, GraphListenerShutdownError):
 * that is no server, not the end of the process.
 *
 * @return Whether the server is there.
 */
template<typename WaitT>
bool wait_for_server(WaitT && wait)
{
  try {
    return wait();
  } catch (const std::runtime_error &) {
    return false;
  }
}

/**
 * @brief Waits for the server of a client with wait_for_service, as wait_for_server(wait).
 */
template<typename ClientT>
bool wait_for_server(const std::shared_ptr<ClientT> & client, std::chrono::nanoseconds timeout)
{
  return wait_for_server([&]() {return client->wait_for_service(timeout);});
}

/**
 * @brief Call a service spinning node until the response arrives.
 *
 * @return The response, or nullptr if the service is not available within service_timeout or
 *         does not answer within response_timeout.
 */
template<typename ServiceT>
typename ServiceT::Response::SharedPtr
call_service(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rclcpp::Client<ServiceT>> client,
  typename ServiceT::Request::SharedPtr request,
  std::chrono::nanoseconds service_timeout = std::chrono::seconds(5),
  std::chrono::nanoseconds response_timeout = std::chrono::seconds(1))
{
  // rclcpp shutting down throws from the graph while we wait: no answer, not the end of the
  // process
  try {
    if (!client->wait_for_service(service_timeout)) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(
          node->get_logger(), client->get_service_name() << ": service not available");
      }
      return nullptr;
    }

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, response_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // Otherwise the late response would stay pending forever
      client->remove_pending_request(future);
      if (rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(node->get_logger(), client->get_service_name() << ": no response");
      }
      return nullptr;
    }

    return future.get();
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(node->get_logger(), client->get_service_name() << ": " << e.what());
    }
    return nullptr;
  }
}

}  // namespace plansys2

#endif  // PLANSYS2_CORE__SERVICECALL_HPP_
