// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

namespace plansys2
{

ApplyAtEndEffect::ApplyAtEndEffect(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  action_graph_ = config().blackboard->get<Graph::Ptr>("action_graph");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
}

BT::NodeStatus
ApplyAtEndEffect::tick()
{
  std::string action;
  getInput("action", action);

  auto effect = (*action_map_)[action].durative_action_info->at_end_effects;

  if (!(*action_map_)[action].at_end_effects_applied) {
    (*action_map_)[action].at_end_effects_applied = true;
    auto current_time = (*action_map_)[action].action_executor->get_current_time();
    auto start_time = (*action_map_)[action].action_executor->get_start_time();
    auto time_from_start = current_time.seconds() - start_time.seconds();
    (*action_map_)[action].at_end_effects_applied_time = time_from_start;
    std::cerr << "*** *** ApplyAtEndEffect: " << action << " at time " << time_from_start << " *** ***" << std::endl;
    apply(effect, problem_client_, 0);
  }

  std::cerr << "*** *** ApplyAtEndEffect: " << action << " SUCCESS *** ***" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

void
ApplyAtEndEffect::propagate(const Graph::Ptr stn)
{
  // Compute the distance matrix.
  Eigen::MatrixXf dist = get_distance_matrix(stn);

  // Check if STN is consistent.
  for (size_t i = 0; i < dist.rows(); i++) {
    if (dist(i,i) < 0) {
      std::cerr << "STN is not consistent!" << std::endl;
      return;
    }
  }

  std::cerr << dist << std::endl;

  // Update the STN.
  for (auto node : stn->nodes) {
    // Create a set to hold the updated output arcs.
    std::set<std::tuple<Node::Ptr, double, double>> new_output_arcs;

    // Iterate over the output arcs.
    auto row = node->node_num;
    for (auto arc : node->output_arcs) {
      auto child = std::get<0>(arc);
      auto col = child->node_num;

      // Save the updated output arc.
      new_output_arcs.insert(std::make_tuple(child, -dist(col, row), dist(row,col)));

      // Find the corresponding input arc.
      std::tuple<Node::Ptr, double, double> input_arc;
      std::set<std::tuple<Node::Ptr, double, double>>::iterator iter;
      iter = child->input_arcs.find(std::make_tuple(node, std::get<1>(arc), std::get<2>(arc)));
      if (iter != child->input_arcs.end())
      {
        input_arc = *iter;
      } else {
        std::cerr << "Input arc not found!" << std::endl;
      }

      // Erase the existing child input arc.
      child->input_arcs.erase(input_arc);

      // Insert the updated child input arc.
      child->input_arcs.insert(std::make_tuple(node, -dist(col, row), dist(row,col)));
    }
    // Replace the output arcs.
    node->output_arcs = new_output_arcs;
  }
}

Eigen::MatrixXf
ApplyAtEndEffect::get_distance_matrix(const Graph::Ptr stn) const
{
  // Initialize the distance matrix as infinity.
  Eigen::MatrixXf dist = std::numeric_limits<float>::infinity() *
    Eigen::MatrixXf::Ones(stn->nodes.size(), stn->nodes.size());

  // Extract the distances imposed by the STN.
  for (const auto node : stn->nodes) {
    auto row = node->node_num;
    for (const auto arc : node->output_arcs) {
      auto child = std::get<0>(arc);
      auto col = child->node_num;
      dist(row, col) = std::get<2>(arc);
      dist(col, row) = -std::get<1>(arc);
    }
  }

  // Solve the all-pairs shortest path problem.
  floyd_warshall(dist);

  return dist;
}

void
ApplyAtEndEffect::floyd_warshall(Eigen::MatrixXf & dist) const
{
  for (size_t k = 0; k < dist.rows(); k++) {
    for (size_t i = 0; i < dist.rows(); i++) {
      for (size_t j = 0; j < dist.rows(); j++) {
        if (dist(i,k) == std::numeric_limits<float>::infinity() ||
            dist(k,j) == std::numeric_limits<float>::infinity()) {
          continue;
        }
        if (dist(i,j) > (dist(i,k) + dist(k,j))) {
          dist(i,j) = dist(i,k) + dist(k,j);
        }
      }
    }
  }
}

}  // namespace plansys2
