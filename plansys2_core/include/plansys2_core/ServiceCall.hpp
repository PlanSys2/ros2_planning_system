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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_core/ClientNode.hpp"
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

// What a component answers (error_info) to a request it does not take because it is not active
constexpr char kNotActive[] = "Requesting service in non-active state";

// Whether a response says that its component is not active (responses without error_info never)
template<typename ResponseT, typename = void>
struct HasErrorInfo : std::false_type {};
template<typename ResponseT>
struct HasErrorInfo<ResponseT, std::void_t<decltype(std::declval<ResponseT>().error_info)>>
  : std::true_type {};

template<typename ResponseT>
bool says_not_active(const ResponseT & response)
{
  if constexpr (HasErrorInfo<ResponseT>::value) {
    return response.error_info == kNotActive;
  } else {
    return false;
  }
}

// Whether a response has a success field
template<typename ResponseT, typename = void>
struct HasSuccess : std::false_type {};
template<typename ResponseT>
struct HasSuccess<ResponseT, std::void_t<decltype(std::declval<ResponseT>().success)>>
  : std::true_type {};

// A response that says the request failed, and why when it has error_info
template<typename ResponseT>
ResponseT failed_response(const std::string & error)
{
  ResponseT response;
  if constexpr (HasSuccess<ResponseT>::value) {
    response.success = false;
  }
  if constexpr (HasErrorInfo<ResponseT>::value) {
    response.error_info = error;
  }
  return response;
}

/**
 * @brief A service callback that answers instead of letting an exception out: one from
 * hostile input (the PDDL parser throws) would end the whole process, and in the monolithic
 * bringup the four components.
 */
template<typename ServiceT, typename CallbackT>
auto guarded(const rclcpp::Logger & logger, CallbackT callback)
{
  return [logger, callback = std::move(callback)](
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<typename ServiceT::Request> request,
    std::shared_ptr<typename ServiceT::Response> response) {
           try {
             callback(header, request, response);
           } catch (const std::exception & e) {
             RCLCPP_ERROR(logger, "Request rejected: %s", e.what());
             *response = failed_response<typename ServiceT::Response>(e.what());
           }
         };
}

/**
 * @brief As guarded(), for a callback that answers later with send_response.
 */
template<typename ServiceT, typename CallbackT>
auto guarded_deferred(const rclcpp::Logger & logger, CallbackT callback)
{
  return [logger, callback = std::move(callback)](
    const std::shared_ptr<rclcpp::Service<ServiceT>> service,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<typename ServiceT::Request> request) {
           try {
             callback(service, header, request);
           } catch (const std::exception & e) {
             RCLCPP_ERROR(logger, "Request rejected: %s", e.what());
             auto response = failed_response<typename ServiceT::Response>(e.what());
             service->send_response(*header, response);
           }
         };
}

/**
 * @brief Whether a component of PlanSys2 is active, as its clients wait for it.
 *
 * A call of a PlanSys2 client waits until its component is active, however long it takes while
 * ROS runs, and goes on as soon as it is: an application may start before PlanSys2. Once seen
 * active it is not asked again, until one of its services goes or stops answering.
 */
class ActiveComponent
{
public:
  ActiveComponent(std::shared_ptr<ClientNode> node, const std::string & component)
  : node_(node), component_(component),
    client_(node->create_client<lifecycle_msgs::srv::GetState>(component + "/get_state"))
  {
  }

  /**
   * @brief Waits until the component is active: false only if ROS shuts down, or the client is
   * interrupted, first.
   */
  bool wait()
  {
    if (active_) {
      return true;
    }
    // One thread asks; the others find it active when they get the lock
    std::lock_guard<std::mutex> lock(mutex_);
    const auto since = std::chrono::steady_clock::now();
    auto last_log = since;
    while (!active_ && node_->running()) {
      // Given up once: the calls after ask once, and do not wait again, until it is back
      const auto limit = gave_up_ ? std::chrono::milliseconds(0) : max_wait_;
      if (max_wait_.count() > 0 && std::chrono::steady_clock::now() - since >= limit) {
        if (!gave_up_) {
          RCLCPP_ERROR(
            node_->get_logger(), "%s is not active after %.0f s: giving up", component_.c_str(),
            std::chrono::duration<double>(max_wait_).count());
        }
        gave_up_ = true;
        // Asked at most once a second meanwhile: each call fails at once
        const auto now = std::chrono::steady_clock::now();
        if (now - last_ask_ < std::chrono::seconds(1)) {
          return false;
        }
        last_ask_ = now;
        if (!ask()) {
          return false;
        }
        active_ = true;
        gave_up_ = false;
        break;
      }
      // At most one question every kPeriod, also when a wait ends at once (shutdown, no node)
      const auto next = std::chrono::steady_clock::now() + kPeriod;
      last_ask_ = std::chrono::steady_clock::now();
      if (ask()) {
        active_ = true;
        gave_up_ = false;
        break;
      }
      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() - last_log > kLogPeriod) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for %s to be active...", component_.c_str());
        last_log = std::chrono::steady_clock::now();
      }
    }
    return active_;
  }

  /**
   * @brief A service of the component went or stopped answering: wait for it again.
   */
  void lost() {active_ = false;}

  /**
   * @brief How long a wait goes on before it gives up: zero, the default, for ever.
   */
  void set_max_wait(std::chrono::milliseconds max_wait) {max_wait_ = max_wait;}

  const std::string & component() const {return component_;}

private:
  // Whether it answers that it is active
  bool ask()
  {
    if (!wait_for_server(client_, std::chrono::seconds(1))) {
      return false;
    }
    try {
      auto future = client_->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (node_->wait(future, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {
        return future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      }
      // A lost answer: a new client, discovered again (see ServiceClient)
      client_->remove_pending_request(future);
      client_ = node_->create_client<lifecycle_msgs::srv::GetState>(component_ + "/get_state");
    } catch (const std::runtime_error &) {
      // ROS shutting down, or the node gone
    }
    return false;
  }

  static constexpr std::chrono::milliseconds kPeriod {500};
  static constexpr std::chrono::seconds kLogPeriod {5};
  std::shared_ptr<ClientNode> node_;
  std::string component_;
  std::chrono::milliseconds max_wait_ {0};
  bool gave_up_ {false};
  std::chrono::steady_clock::time_point last_ask_;
  std::atomic<bool> active_ {false};
  std::mutex mutex_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_;
};

/**
 * @brief A service client of a PlanSys2 client, which makes itself anew when an answer is lost.
 *
 * A request that gets no answer in time is sent again with a new rclcpp client: new endpoints,
 * discovered again. Some middlewares lose every answer to a client whose endpoints were matched
 * badly while many processes started at once (Fast DDS, 2026-10-19), and waiting longer on the
 * same client does not help. With more than one attempt, only for services whose requests may be
 * repeated: queries, and changes that leave the same state when repeated. A call that ends
 * without answer leaves a new client for the next one, whatever its attempts.
 *
 * With an ActiveComponent, a call first waits until the component is active, and its server
 * gone or a call without answer make the next one wait for it again. An answer saying that the
 * component is not active (kNotActive) makes the call wait for it and send the request again.
 */
template<typename ServiceT>
class ServiceClient
{
public:
  using SharedPtr = std::shared_ptr<ServiceClient<ServiceT>>;
  using Response = typename ServiceT::Response::SharedPtr;
  using Request = typename ServiceT::Request::SharedPtr;

  ServiceClient(
    std::shared_ptr<ClientNode> node, const std::string & name,
    std::shared_ptr<ActiveComponent> active = nullptr)
  : node_(node), name_(name), active_(active), client_(node->create_client<ServiceT>(name))
  {
  }

  /**
   * @brief Calls the service.
   *
   * @param[in] request The request.
   * @param[in] response_timeout How long each attempt waits for the answer.
   * @param[in] attempts How many times the request is sent, each with a new client but the first.
   * @param[in] service_timeout How long to wait for the server (with an ActiveComponent, once it
   *            is active; without it, the server is waited for again).
   * @return The response, or nullptr without server or without answer (or if ROS shuts down).
   */
  Response call(
    const Request & request,
    std::chrono::nanoseconds response_timeout = std::chrono::seconds(1),
    int attempts = 3,
    std::chrono::nanoseconds service_timeout = std::chrono::seconds(5))
  {
    bool waited_again = false;
    int not_active_waits = 0;
    for (int attempt = 1; attempt <= attempts && node_->running(); attempt++) {
      if (active_ && !active_->wait()) {
        return nullptr;
      }
      auto client = current();
      if (!wait_for_server(client, service_timeout)) {
        if (active_ && !waited_again && node_->running()) {
          // Gone (PlanSys2 stopped or restarted): back to waiting for it, once; active and still
          // without this service, it is an error
          active_->lost();
          waited_again = true;
          attempt--;
          continue;
        }
        if (node_->running()) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), name() << ": service not available");
        }
        return nullptr;
      }
      try {
        auto future = client->async_send_request(request);
        if (node_->wait(future, response_timeout) == rclcpp::FutureReturnCode::SUCCESS) {
          auto response = future.get();
          if (active_ && says_not_active(*response) && not_active_waits++ < kNotActiveWaits) {
            // Deactivated since it was seen active: wait for it again, and ask again
            active_->lost();
            attempt--;
            continue;
          }
          return response;
        }
        // Otherwise the late answer would stay pending forever
        client->remove_pending_request(future);
      } catch (const std::exception & e) {
        // rclcpp shutting down throws from the graph: no answer, not the end of the process
        if (node_->running()) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), name() << ": " << e.what());
        }
        return nullptr;
      }
      if (!node_->running()) {
        return nullptr;
      }
      if (!renew(client)) {
        return nullptr;
      }
      if (attempt < attempts) {
        RCLCPP_WARN_STREAM(
          node_->get_logger(), name() << ": no response, sending it again with a new client (" <<
            attempt + 1 << "/" << attempts << ")");
      }
    }
    if (active_) {
      active_->lost();  // The next call checks that it is still active
    }
    if (node_->running()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), name() << ": no response");
    }
    return nullptr;
  }

  /**
   * @brief Whether the server is there, waiting up to timeout for it.
   */
  bool wait_for_service(std::chrono::nanoseconds timeout)
  {
    return wait_for_server(current(), timeout);
  }

  bool service_is_ready() const {return current()->service_is_ready();}

  std::string name() const {return current()->get_service_name();}

private:
  typename rclcpp::Client<ServiceT>::SharedPtr current() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return client_;
  }

  // A new client, unless another call already made one after this one failed
  bool renew(const typename rclcpp::Client<ServiceT>::SharedPtr & failed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (client_ != failed) {
      return true;
    }
    try {
      client_ = node_->create_client<ServiceT>(name_);
    } catch (const std::runtime_error &) {
      return false;  // The node is gone
    }
    return true;
  }

  // A component that says it is active and answers that it is not, so many times: an error
  static constexpr int kNotActiveWaits = 3;
  std::shared_ptr<ClientNode> node_;
  std::string name_;
  std::shared_ptr<ActiveComponent> active_;
  mutable std::mutex mutex_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace plansys2

#endif  // PLANSYS2_CORE__SERVICECALL_HPP_
