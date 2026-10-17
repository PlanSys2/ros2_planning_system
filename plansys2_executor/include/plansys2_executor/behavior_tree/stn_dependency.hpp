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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__STN_DEPENDENCY_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__STN_DEPENDENCY_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

using ActionMap = std::map<std::string, ActionExecutionInfo>;

// The STN builder writes "<child id> <START|END> <parent id> <INIT|START|END>"
struct StnDependency
{
  std::string child_id;
  std::string child_type;
  std::string parent_id;
  std::string parent_type;
};

// The dependency in an STN node attribute; nullopt for a plain action id (SimpleBTBuilder)
std::optional<StnDependency> parse_stn_dependency(const std::string & text);

// The graph node of an action event, nullptr when it is not in the graph
Node::Ptr find_stn_node(
  const Graph::Ptr & graph, const std::string & action_id, const std::string & type);

// Seconds since the parent event of a dependency; nullopt if it has not happened yet
std::optional<double> seconds_since_parent(
  ActionMap & action_map, const StnDependency & dependency, const rclcpp::Time & now);

// Time bounds of the arc between the two events, [0, inf) when unknown
std::pair<double, double> stn_bounds(const Graph::Ptr & graph, const StnDependency & dependency);

// An action event happened at `now`: its input arcs take the actual elapsed times and the
// STN is propagated again. False if the STN becomes inconsistent.
bool update_stn_on_event(
  const Graph::Ptr & graph, const std::shared_ptr<BTBuilder> & builder,
  ActionMap & action_map, const std::string & action_id, ActionType type,
  const rclcpp::Time & now);

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__STN_DEPENDENCY_HPP_
