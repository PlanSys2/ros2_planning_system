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

#ifndef PLANSYS2_CORE__CLIENTNODE_HPP_
#define PLANSYS2_CORE__CLIENTNODE_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

namespace plansys2
{

// Threads of the executor of each component: callbacks that wait for clients, and their answers
constexpr size_t kComponentThreads = 4;

// The QoS of services and parameter clients: rclcpp takes rclcpp::QoS from Iron on, an rmw
// profile before
#if RCLCPP_VERSION_MAJOR >= 21
inline rclcpp::QoS services_qos() {return rclcpp::ServicesQoS();}
inline rclcpp::QoS parameters_qos() {return rclcpp::ParametersQoS();}
#else
inline const rmw_qos_profile_t & services_qos() {return rmw_qos_profile_services_default;}
inline const rmw_qos_profile_t & parameters_qos() {return rmw_qos_profile_parameters;}
#endif

/**
 * @brief The node a PlanSys2 client works on, which it does not own: one of a component or of
 * an application.
 *
 * The clients and subscriptions of the client go in a callback group of their own, which the
 * executor of the node does not take: the client serves it, and only it, while it waits for an
 * answer (serve() brings what came meanwhile). So it calls from anywhere: before the node is
 * spun, or from a callback of any executor, single threaded too.
 *
 * It keeps the node only through weak pointers: the node, or the component that has it, may
 * keep the client without a cycle; once the node is gone, calls get no answer.
 */
class ClientNode
{
public:
  using Interfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeGraphInterface,
    rclcpp::node_interfaces::NodeServicesInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeWaitablesInterface>;

  explicit ClientNode(Interfaces node)
  : base_(node.get_node_base_interface()),
    graph_(node.get_node_graph_interface()),
    services_(node.get_node_services_interface()),
    topics_(node.get_node_topics_interface()),
    logging_(node.get_node_logging_interface()),
    clock_interface_(node.get_node_clock_interface()),
    parameters_(node.get_node_parameters_interface()),
    waitables_(node.get_node_waitables_interface()),
    logger_(node.get_node_logging_interface()->get_logger()),
    clock_(node.get_node_clock_interface()->get_clock()),
    name_(node.get_node_base_interface()->get_name()),
    namespace_(node.get_node_base_interface()->get_namespace()),
    group_(node.get_node_base_interface()->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false)),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        executor_options(node)))
  {
    executor_->add_callback_group(group_, node.get_node_base_interface());
  }

  ClientNode(const ClientNode &) = delete;
  ClientNode & operator=(const ClientNode &) = delete;

  /**
   * @brief The interfaces of the node while it exists, to create what the helpers here do not.
   */
  std::optional<Interfaces> lock() const
  {
    auto base = base_.lock();
    auto graph = graph_.lock();
    auto services = services_.lock();
    auto topics = topics_.lock();
    auto logging = logging_.lock();
    auto clock = clock_interface_.lock();
    auto parameters = parameters_.lock();
    auto waitables = waitables_.lock();
    if (!base || !graph || !services || !topics || !logging || !clock || !parameters ||
      !waitables)
    {
      return std::nullopt;
    }
    return Interfaces(base, graph, services, topics, logging, clock, parameters, waitables);
  }

  rclcpp::CallbackGroup::SharedPtr group() const {return group_;}
  const rclcpp::Logger & get_logger() const {return logger_;}
  rclcpp::Time now() const {return clock_->now();}
  const std::string & get_name() const {return name_;}
  const std::string & get_namespace() const {return namespace_;}

  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr create_client(const std::string & name)
  {
    auto node = lock_or_throw();
    return rclcpp::create_client<ServiceT>(
      node.get_node_base_interface(), node.get_node_graph_interface(),
      node.get_node_services_interface(), name, services_qos(), group_);
  }

  template<typename MessageT, typename CallbackT>
  typename rclcpp::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic, const rclcpp::QoS & qos, CallbackT && callback)
  {
    auto node = lock_or_throw();
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    auto parameters = node.get_node_parameters_interface();
    auto topics = node.get_node_topics_interface();
    return rclcpp::create_subscription<MessageT>(
      parameters, topics, topic, qos, std::forward<CallbackT>(callback), options);
  }

  /**
   * @brief Waits until the future is ready, the timeout passes or ROS shuts down, serving the
   * group of the client meanwhile.
   */
  template<typename FutureT>
  rclcpp::FutureReturnCode wait(FutureT & future, std::chrono::nanoseconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (running()) {
      const auto left = end - std::chrono::steady_clock::now();
      if (left <= std::chrono::nanoseconds::zero()) {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready ?
               rclcpp::FutureReturnCode::SUCCESS : rclcpp::FutureReturnCode::TIMEOUT;
      }
      // In slices, to see a shutdown
      const auto slice = std::min<std::chrono::nanoseconds>(left, kSlice);
      std::unique_lock<std::mutex> lock(serve_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (executor_->spin_until_future_complete(future, slice) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          return rclcpp::FutureReturnCode::SUCCESS;
        }
      } else if (future.wait_for(slice) == std::future_status::ready) {
        // Another thread of the client serves the group, and brought it
        return rclcpp::FutureReturnCode::SUCCESS;
      }
    }
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }

  /**
   * @brief While interrupted, the waits of the client end at once, as with a shutdown: for the
   * owner that stops a thread which may be in a call.
   */
  void interrupt(bool interrupted) {interrupted_ = interrupted;}

  /**
   * @brief Whether a wait may go on: ROS runs and the client is not interrupted.
   */
  bool running() const {return rclcpp::ok() && !interrupted_;}

  /**
   * @brief Brings what came for the group without waiting: messages, feedback and results.
   */
  void serve() const
  {
    std::unique_lock<std::mutex> lock(serve_mutex_, std::try_to_lock);
    if (lock.owns_lock() && rclcpp::ok()) {
      // All that is queued: spin_some takes one message of each subscription
      executor_->spin_all(kSlice);
    }
  }

private:
  static rclcpp::ExecutorOptions executor_options(Interfaces node)
  {
    rclcpp::ExecutorOptions options;
    options.context = node.get_node_base_interface()->get_context();  // The one of the node
    return options;
  }

  Interfaces lock_or_throw() const
  {
    auto node = lock();
    if (!node) {
      throw std::runtime_error("The node of the PlanSys2 client " + name_ + " is gone");
    }
    return *node;
  }

  std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_;
  std::weak_ptr<rclcpp::node_interfaces::NodeGraphInterface> graph_;
  std::weak_ptr<rclcpp::node_interfaces::NodeServicesInterface> services_;
  std::weak_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_;
  std::weak_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_;
  std::weak_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface_;
  std::weak_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_;
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> waitables_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string namespace_;
  rclcpp::CallbackGroup::SharedPtr group_;
  // Serves only the group, and only while the client waits; after the group, gone before it
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  mutable std::mutex serve_mutex_;
  std::atomic<bool> interrupted_ {false};
  static constexpr std::chrono::milliseconds kSlice {50};
};

}  // namespace plansys2

#endif  // PLANSYS2_CORE__CLIENTNODE_HPP_
