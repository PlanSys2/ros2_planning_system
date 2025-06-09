// Copyright 2024 Intelligent Robotics Lab
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

#ifndef PLANSYS2_CORE__DERIVED_RESOLUTION_GRAPH_HPP_
#define PLANSYS2_CORE__DERIVED_RESOLUTION_GRAPH_HPP_

#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <iostream>
#include <stack>

#include "plansys2_core/Action.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_core/NodeVariant.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

// class ParentNodesEdgesMap 
// {
// public:
//   // Add, remove, find, or iterate edges with meaningful function names
//   void addEdge(const NodeVariant& parent, const std::string& label, const NodeVariant& child);
//   // ...other methods

// private:
//   std::unordered_map<NodeVariant, std::unordered_map<std::string, std::unordered_set<NodeVariant>>> data_;
// };

class DerivedResolutionGraph
{
public:
  DerivedResolutionGraph() {};

  DerivedResolutionGraph(const std::vector<plansys2_msgs::msg::Derived> & derived_predicates);  // NOLINT

  DerivedResolutionGraph(const std::vector<plansys2::Derived> & derived_predicates);  // NOLINT

  DerivedResolutionGraph(const std::unordered_set<plansys2::Derived> & derived_predicates);  // NOLINT

  void printGraph() const;

  void printGraphLayers() const;

  void addNode(const NodeVariant & node);

  void addEdge(const NodeVariant & u, const NodeVariant & v);
  void addEdgeFromPreconditions(const NodeVariant & node, const plansys2_msgs::msg::Tree & tree);

  // DFS from a given start node
  void depthFirstTraverse(const NodeVariant& start, 
    const std::function<void(const NodeVariant&)>& func,
    std::unordered_set<NodeVariant>& visited,
    bool check_dependencies = false) const;

  void depthFirstTraverse(const NodeVariant& start, 
    const std::function<void(const NodeVariant&)>& func,
    bool check_dependencies = false) const;

  void depthFirstTraverseFromNodes(
    const std::function<void(const NodeVariant&)>& func, 
    bool check_dependencies = false,
    const std::vector<NodeVariant>& start_nodes = {},
    bool check_visited = true) const;

  void depthFirstTraverseAll(
    const std::function<void(const NodeVariant&)>& func,
    bool check_dependencies = false) const;

  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst(
    const std::vector<NodeVariant>& start_nodes = {}) const;

  std::deque<plansys2::Derived> getDerivedPredicatesFromActions(
    const std::vector<plansys2::ActionVariant> & actions) const;

  void backtrackTraverse(const NodeVariant & start, const std::function<void(const NodeVariant&)>& func) const;

  void backtrackTraverse(
    const NodeVariant & node, std::unordered_set<NodeVariant> & visited, 
    const std::function<void(const NodeVariant&)>& func) const;

  DerivedResolutionGraph getSubGraphFromNodes(const std::vector<NodeVariant> & nodes) const;

  plansys2::DerivedResolutionGraph pruneGraphToActions(const std::vector<plansys2::ActionVariant> & actions);

  void appendActions(const std::vector<plansys2::ActionVariant> & actions);
  void appendAction(const plansys2::ActionVariant & action);

  std::vector<std::vector<Derived>> computeSCCsTarjanDerivedPredicates() const;
  void strongConnect(
    const Derived& node,
    int& current_index,
    std::unordered_map<Derived, int>& index,
    std::unordered_map<Derived, int>& lowlink,
    std::unordered_set<Derived>& on_stack,
    std::stack<Derived>& stack,
    std::vector<std::vector<Derived>>& sccs) const;

  const std::unordered_set<NodeVariant>& getNodeOutEdges(const NodeVariant & node) {return adj_list_[node];}
  const std::unordered_set<NodeVariant>& getNodeInEdges(const NodeVariant & node) {return parent_nodes_[node];}
  const std::unordered_set<NodeVariant>& getRoots() const { return roots_; }
  const std::unordered_set<NodeVariant>& getNodes() const { return nodes_; }
  
  auto getEdgeNumber() const {return edge_count_;}
  auto getNodeNumber() const {return nodes_.size();}
  auto getRootNumber() const {return roots_.size();} 
  auto getDerivedPredicatesNumber() const {return derived_predicates_.size();}
  auto getActionsNumber() const {return actions_.size();}

  std::vector<std::string> getNodesNames() const;
  std::vector<std::string> getPredicatesNames() const;
  std::vector<std::string> getFunctionsNames() const;
  std::vector<std::string> getDerivedPredicatesNames() const;

  std::vector<plansys2::Predicate> getPredicates() const;
  std::vector<plansys2::Function> getFunctions() const;
  std::unordered_set<plansys2::Derived> getDerivedPredicates() const {return derived_predicates_;}
  std::unordered_set<plansys2::ActionVariant> getActions() const {return actions_;}
  
  std::vector<std::string> getRootsNames() const;
  std::unordered_set<NodeVariant> getParentNodes(const NodeVariant& node) const;
  std::vector<std::string> getParentNodesNames(const NodeVariant& node) const;

  void exportToDOT(const std::string& filename) const;

  void clear() {adj_list_.clear(); parent_nodes_.clear(); nodes_.clear(); roots_.clear(); edge_count_ = 0;}

  bool operator==(const DerivedResolutionGraph& other) const
  {
    if (this == &other) return true;
    return adj_list_ == other.adj_list_
        && parent_nodes_ == other.parent_nodes_
        && nodes_ == other.nodes_
        && roots_ == other.roots_
        && derived_predicates_ == other.derived_predicates_
        && actions_ == other.actions_
        && node_ids_ == other.node_ids_
        && edge_count_ == other.edge_count_;
  }

  using NodeEdgesMap = std::unordered_map<NodeVariant, std::unordered_set<NodeVariant>>;

private:
  NodeEdgesMap adj_list_;
  NodeEdgesMap parent_nodes_;
  std::unordered_set<NodeVariant> roots_;
  std::unordered_set<NodeVariant> nodes_;
  std::unordered_map<NodeVariant, size_t> node_ids_;
  size_t next_id_ = 0;
  
  std::unordered_set<plansys2::Derived> derived_predicates_;
  std::unordered_set<plansys2::ActionVariant> actions_;
  
  size_t edge_count_ = 0;

  void invertedDfsHelper(const NodeVariant& node, 
    const std::function<void(const NodeVariant&)>& func,
    std::unordered_set<NodeVariant>& visited) const;

  void dfsHelper(const NodeVariant& node, 
    const std::function<void(const NodeVariant&)>& func,
    std::unordered_set<NodeVariant>& visited,
    bool check_dependencies = false) const;

  bool parentsVisited(const NodeVariant& node, const std::unordered_set<NodeVariant>& visited) const;

  friend struct std::hash<DerivedResolutionGraph>;
};

}  // namespace plansys2

namespace std
{
template <typename NodeEdgesMap>
std::size_t unordered_nodeedgesmap_hash(const NodeEdgesMap& map)
{
    std::size_t seed = 0;
    for (const auto& [key, neighbors] : map) {
        // Hash the key
        std::size_t entry_hash = std::hash<std::decay_t<decltype(key)>>{}(key);
        // Hash the neighbor set (order-independent)
        entry_hash ^= unordered_container_hash(neighbors);
        // XOR with seed (order-independent for map)
        seed ^= entry_hash;
    }
    // Mix in size for extra entropy
    seed ^= map.size();
    return seed;
}

template<>
struct hash<plansys2::DerivedResolutionGraph>
{
  std::size_t operator()(const plansys2::DerivedResolutionGraph & graph) const noexcept
  {
    std::size_t seed = 0;
    hash_combine(seed, graph.edge_count_);
    seed ^= unordered_container_hash(graph.nodes_);
    seed ^= unordered_container_hash(graph.roots_);
    seed ^= unordered_container_hash(graph.derived_predicates_);
    seed ^= unordered_container_hash(graph.actions_);
    seed ^= unordered_container_hash(graph.node_ids_);
    seed ^= unordered_nodeedgesmap_hash(graph.adj_list_);
    seed ^= unordered_nodeedgesmap_hash(graph.parent_nodes_);
    return seed;
  }
};
}  // namespace std

#endif  // PLANSYS2_CORE__DERIVED_RESOLUTION_GRAPH_HPP_
