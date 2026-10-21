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

#ifndef PLANSYS2_CORE__NODENAMES_HPP_
#define PLANSYS2_CORE__NODENAMES_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @brief Asks the graph for the node names without racing with rclcpp shutting down.
 *
 * Shutdown waits for a query in progress and no query starts after it: the context never goes
 * away in the middle of one.
 */
class NodeNames
{
public:
  explicit NodeNames(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base)
  : graph_context_(base->get_context()), state_(std::make_shared<State>())
  {
    std::weak_ptr<State> weak = state_;
    handle_ = graph_context_->add_pre_shutdown_callback(
      [weak]() {
        if (auto state = weak.lock()) {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->down = true;
        }
      });
  }

  ~NodeNames()
  {
    graph_context_->remove_pre_shutdown_callback(handle_);
  }

  NodeNames(const NodeNames &) = delete;
  NodeNames & operator=(const NodeNames &) = delete;

  /**
   * @brief The fully qualified names of the nodes in the graph.
   * @return Nothing once rclcpp shuts down or the query fails.
   */
  std::optional<std::vector<std::string>>
  get(rclcpp::node_interfaces::NodeGraphInterface & graph)
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->down || !graph_context_->is_valid()) {
      return std::nullopt;
    }
    try {
      return graph.get_node_names();
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

private:
  struct State
  {
    std::mutex mutex;
    bool down {false};
  };

  rclcpp::Context::SharedPtr graph_context_;
  std::shared_ptr<State> state_;
  rclcpp::PreShutdownCallbackHandle handle_;
};

}  // namespace plansys2

#endif  // PLANSYS2_CORE__NODENAMES_HPP_
