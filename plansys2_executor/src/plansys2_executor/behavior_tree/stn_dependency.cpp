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

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <utility>

#include "plansys2_executor/behavior_tree/stn_dependency.hpp"

namespace plansys2
{

std::optional<StnDependency>
parse_stn_dependency(const std::string & text)
{
  // Action ids end in ":<time>"; the INIT pseudo-action is ":0"
  static const std::regex dependency_regex(
    R"(^(.*?:\d+) (START|END) (.*:\d+) (INIT|START|END)$)");
  std::smatch match;
  if (!std::regex_match(text, match, dependency_regex)) {
    return std::nullopt;
  }
  return StnDependency{match[1].str(), match[2].str(), match[3].str(), match[4].str()};
}

Node::Ptr
find_stn_node(const Graph::Ptr & graph, const std::string & action_id, const std::string & type)
{
  if (!graph) {
    return nullptr;
  }
  auto it = std::find_if(
    graph->nodes.begin(), graph->nodes.end(),
    [&](const Node::Ptr & node) {
      return BTBuilder::to_action_id(node->action, graph->action_time_precision) == action_id &&
             BTBuilder::to_string(node->action.type) == type;
    });
  return it == graph->nodes.end() ? nullptr : *it;
}

namespace
{

// When the event of an action happened, if it did
std::optional<rclcpp::Time> event_time(
  ActionMap & action_map, const std::string & action_id, const std::string & type)
{
  auto it = action_map.find(action_id);
  if (it == action_map.end() || it->second.action_executor == nullptr) {
    return std::nullopt;
  }
  const auto & info = it->second;
  if (type == "INIT") {
    return info.at_end_effects_applied_time;
  }
  if (type == "START" && info.at_start_effects_applied) {
    return info.at_start_effects_applied_time;
  }
  if (type == "END" && info.at_end_effects_applied && info.action_executor->is_finished()) {
    return info.at_end_effects_applied_time;
  }
  return std::nullopt;
}

}  // namespace

std::optional<double>
seconds_since_parent(
  ActionMap & action_map, const StnDependency & dependency, const rclcpp::Time & now)
{
  auto parent_time = event_time(action_map, dependency.parent_id, dependency.parent_type);
  if (!parent_time) {
    return std::nullopt;
  }
  // Plain seconds: subtracting times of different clock types throws
  return now.seconds() - parent_time->seconds();
}

std::pair<double, double>
stn_bounds(const Graph::Ptr & graph, const StnDependency & dependency)
{
  const std::pair<double, double> unbounded{0.0, std::numeric_limits<double>::infinity()};
  auto child = find_stn_node(graph, dependency.child_id, dependency.child_type);
  auto parent = find_stn_node(graph, dependency.parent_id, dependency.parent_type);
  if (!child || !parent) {
    return unbounded;
  }
  for (const auto & [node, lower, upper] : parent->output_arcs) {
    if (node == child) {
      return {lower, upper};
    }
  }
  return unbounded;
}

bool
update_stn_on_event(
  const Graph::Ptr & graph, const std::shared_ptr<BTBuilder> & builder,
  ActionMap & action_map, const std::string & action_id, ActionType type,
  const rclcpp::Time & now)
{
  auto child = find_stn_node(graph, action_id, BTBuilder::to_string(type));
  if (!child || !builder) {
    return true;  // Not an STN plan
  }

  std::set<std::tuple<Node::Ptr, double, double>> input_arcs;
  for (const auto & arc : child->input_arcs) {
    auto parent = std::get<0>(arc);
    auto parent_time = event_time(
      action_map, BTBuilder::to_action_id(parent->action, graph->action_time_precision),
      BTBuilder::to_string(parent->action.type));
    if (!parent_time) {
      input_arcs.insert(arc);  // Its parent has not happened: nothing to fix yet
      continue;
    }

    // The arc now holds what actually happened
    double elapsed = now.seconds() - parent_time->seconds();
    input_arcs.insert(std::make_tuple(parent, elapsed, elapsed));
    for (auto it = parent->output_arcs.begin(); it != parent->output_arcs.end(); ++it) {
      if (std::get<0>(*it) == child) {
        parent->output_arcs.erase(it);
        break;
      }
    }
    parent->output_arcs.insert(std::make_tuple(child, elapsed, elapsed));
  }
  child->input_arcs = input_arcs;

  return builder->propagate(graph);
}

}  // namespace plansys2
