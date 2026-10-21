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

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/ServiceCall.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

namespace plansys2
{

/**
 * @class plansys2::LifecycleServiceClient
 * @brief Asks a managed node for its lifecycle state and transitions.
 *
 * It works on the node of the manager, which it does not own: it creates no node. Its calls wait
 * for their answers serving only its own callbacks (plansys2::ClientNode).
 */
class LifecycleServiceClient
{
public:
  /**
   * @param[in] node The node of the manager.
   * @param[in] managed_node The node whose lifecycle it manages.
   */
  LifecycleServiceClient(std::shared_ptr<ClientNode> node, const std::string & managed_node);

  /**
   * @brief The current state of the managed node, or PRIMARY_STATE_UNKNOWN without answer.
   */
  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));

  /**
   * @brief Triggers a transition of the managed node: whether it succeeded.
   */
  bool change_state(
    std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds(3));

  rclcpp::Logger get_logger() const {return node_->get_logger();}

private:
  std::shared_ptr<ClientNode> node_;
  std::string managed_node_;
  ServiceClient<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  ServiceClient<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
};

/**
 * @brief Starts up all managed nodes using their LifecycleServiceClient interfaces.
 *
 * Configures planner, domain_expert, problem_expert and executor, then activates them,
 * checking each state. Every wait is bounded and stops if ROS shuts down.
 *
 * @param[in] manager_nodes Map of node names to their LifecycleServiceClient instances.
 *   It must contain the four nodes above.
 * @param[in] timeout Timeout for each startup operation.
 * @return true if all nodes started successfully, false otherwise.
 */
bool
startup_function(
  const std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
  std::chrono::seconds timeout);

/**
 * @brief Watches the components once they are up, and acts when one needs it.
 *
 * Each step asks the state of each component. One that comes back unconfigured (its process
 * started again) is configured and activated. One that does not answer for lost_after is lost;
 * lost for longer than grace, the executor is deactivated, and the plans in course fail with
 * a clear result instead of waiting for it. When every component is active again, so is the
 * executor. A component deactivated by someone else is left as it is.
 */
class Supervisor
{
public:
  struct Options
  {
    std::chrono::milliseconds lost_after {3000};
    std::chrono::milliseconds grace {10000};
    std::chrono::seconds transition_timeout {5};
  };

  Supervisor(
    const std::map<std::string, std::shared_ptr<LifecycleServiceClient>> & manager_nodes,
    const Options & options);

  // One look at every component, and what it needs
  void step();

  // Whether the executor is deactivated because a component is gone
  bool degraded() const {return degraded_;}

private:
  struct Watch
  {
    std::chrono::steady_clock::time_point last_seen;
    bool lost {false};
    unsigned int state {lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
  };

  void bring_up(const std::string & name);

  std::map<std::string, std::shared_ptr<LifecycleServiceClient>> nodes_;
  Options options_;
  std::map<std::string, Watch> watch_;
  bool degraded_ {false};
};

}  // namespace plansys2

#endif  // PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
