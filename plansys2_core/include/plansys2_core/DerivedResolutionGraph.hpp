// Copyright 2025 Intelligent Robotics Lab
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

#ifndef PLANSYS2_CORE__DERIVEDRESOLUTIONGRAPH_HPP_
#define PLANSYS2_CORE__DERIVEDRESOLUTIONGRAPH_HPP_

#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "plansys2_core/Action.hpp"
#include "plansys2_core/NodeVariant.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

/**
 * @class DerivedResolutionGraph
 * @brief Represents a directed graph structure for managing derived predicates and their
 * dependencies in a planning system.
 *
 * This class is used to model and resolve derived predicates and its dependencies.
 * Check issue #328 for a more detailed explanation.
 * It provides methods for graph construction, traversal (DFS, backtracking), subgraph
 * extraction, and computing the graph's strongly connected components (SCC).
 * In this graph, nodes in the upper layer (depth 0) represent predicates, functions, or
 * instances. Nodes in the lower layers (depth one or more) represent derived predicates.
 * Nodes in the last layer represent actions.
 * The edges indicate dependencies between the nodes, with arrows pointing from parent to
 * child nodes.
 * More specifically, the graph nodes are instances of the NodeVariant class, which can
 * hold plansys2::Predicate, plansys2::Function, plansys2::Derived, or
 * plansys2::ActionVariant.
 * This definition allows derived predicates to depend on other derived predicates,
 * including recursive self-dependencies, and allows negated derived predicates in their
 * preconditions, provided that such negation does not occur within a recursive cycle.
 *
 * Key Features:
 * - Construction from various collections of derived predicates.
 * - Methods to add nodes and edges, including from action preconditions.
 * - Multiple traversal strategies (depth-first, backtracking, SCC detection).
 * - Extraction of subgraphs and pruning based on actions.
 * - Query functions for nodes, edges, roots, predicates, functions, and actions.
 * - Export capability to DOT format for visualization.
 * - Equality operator for graph comparison.
 *
 * @note NodeVariant, Derived, Predicate, Function, and ActionVariant are types defined in
 * the plansys2 namespace.
 */
class DerivedResolutionGraph
{
public:
  DerivedResolutionGraph() {}

  DerivedResolutionGraph(
    const std::vector<plansys2_msgs::msg::Derived> & derived_predicates);  // NOLINT

  DerivedResolutionGraph(const std::vector<plansys2::Derived> & derived_predicates);  // NOLINT

  DerivedResolutionGraph(
    const std::unordered_set<plansys2::Derived> & derived_predicates);  // NOLINT

  void printGraph() const;

  void printGraphLayers() const;

  void addNode(const NodeVariant & node);

  void addEdge(const NodeVariant & u, const NodeVariant & v);

  /**
   * @brief Adds edges to the resolution graph based on the preconditions of a given node.
   *
   * This method analyzes the preconditions specified in the provided tree and creates
   * corresponding edges in the derived resolution graph, originating from the given node.
   *
   * @param node The node from which edges will be added, represented as a NodeVariant.
   * @param tree The tree structure containing the preconditions to be analyzed.
   */
  void addEdgeFromPreconditions(const NodeVariant & node, const plansys2_msgs::msg::Tree & tree);

  /**
   * @brief Performs a depth first traverse from a given Node and applies func to each node.
   *
   * This method Performs a depth first traverse from a given Node, applies func to each node,
   * and adds each visited node to the visisted vector to avoid visiting the same node multiple
   * times. When check_dependencies is true, it only expands a node when all its parents were
   * already visited.
   *
   * @param start The node from which the traverse should start.
   * @param func The function to apply to each node.
   * @param visited unordered_set containing all nodes that were already visisted
   * @param check_dependencies whether to only expand a node if the parents were visisted
   */
  void depthFirstTraverse(
    const NodeVariant & start, const std::function<void(const NodeVariant &)> & func,
    std::unordered_set<NodeVariant> & visited, bool check_dependencies = false) const;

  /**
   * @brief Performs a depth first traverse from a given Node and applies func to each node.
   *
   * This method Performs a depth first traverse from a given Node and applies func to each node.
   * When check_dependencies is true, it only expands a node when all its parents were
   * already visited.
   *
   * @param start The node from which the traverse should start.
   * @param func The function to apply to each node.
   * @param check_dependencies whether to only expand a node if the parents were visisted
   */
  void depthFirstTraverse(
    const NodeVariant & start, const std::function<void(const NodeVariant &)> & func,
    bool check_dependencies = false) const;

  /**
   * @brief Performs depth first traverse from a given set of Nodes and applies func to each node.
   *
   * This method performs a depth first traverse from a given set of Nodes and applies func to
   * each node.
   * When check_dependencies is true, it only expands a node when all its parents were
   * already visited.
   *
   * @param start_nodes Vector containing the nodes from which the traverse should start.
   * @param func The function to apply to each node.
   * @param check_dependencies whether to only expand a node if the parents were visisted
   */
  void depthFirstTraverseFromNodes(
    const std::function<void(const NodeVariant &)> & func, bool check_dependencies = false,
    const std::vector<NodeVariant> & start_nodes = {}) const;

  /**
   * @brief Performs depth first traverse from all root Nodes and applies func to each node.
   *
   * This method Performs a depth first traverse from all root Nodes and applies func to each node.
   * When check_dependencies is true, it only expands a node when all its parents were
   * already visited.
   *
   * @param func The function to apply to each node.
   * @param check_dependencies whether to only expand a node if the parents were visisted
   */
  void depthFirstTraverseAll(
    const std::function<void(const NodeVariant &)> & func, bool check_dependencies = false) const;

  /**
   * @brief Retrieves a list of derived predicates using a depth-first traversal.
   *
   * This method performs a depth-first search starting from the specified nodes (if any),
   * collecting all reachable derived predicates in the order they are discovered.
   *
   * @param start_nodes Optional vector of starting nodes for the traversal. If empty, the
   *        traversal will start from all root nodes in the graph.
   * @return std::vector<plansys2::Derived> A vector containing the derived predicates
   *        found during traversal.
   */
  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst(
    const std::vector<NodeVariant> & start_nodes = {}) const;

  /**
   * @brief Retrieves a list of derived predicates that given actions depend on.
   *
   * This method returns the derived predicates that the given actions depend on.
   *
   * @param actions A vector containing the action variants to be analyzed.
   * @return A deque of Derived objects representing the derived predicates the actions depend on.
   */
  std::deque<plansys2::Derived> getDerivedPredicatesFromActions(
    const std::vector<plansys2::ActionVariant> & actions) const;

  void backtrackTraverse(
    const NodeVariant & start, const std::function<void(const NodeVariant &)> & func) const;

  void backtrackTraverse(
    const NodeVariant & node, std::unordered_set<NodeVariant> & visited,
    const std::function<void(const NodeVariant &)> & func) const;

  /**
   * @brief Extracts a subgraph from the current DerivedResolutionGraph using the specified nodes.
   *
   * This method creates and returns a new DerivedResolutionGraph that contains the nodes
   * provided in the input vector and its children nodes.
   *
   * @param nodes A vector of NodeVariant objects representing the root nodes to obtain the
   *              subgraph from.
   * @return A DerivedResolutionGraph object representing the subgraph obtained from the
   *         input nodes.
   */
  DerivedResolutionGraph getSubGraphFromNodes(const std::vector<NodeVariant> & nodes) const;

  /**
   * @brief Prunes the graph to include only nodes the specified actions depend on.
   *
   * This function takes a list of actions and returns a new DerivedResolutionGraph
   * that contains only the nodes and edges relevant to those actions. It is useful
   * for focusing the resolution graph on a subset of actions, potentially improving
   * performance.s.
   *
   * @param actions A vector of ActionVariant objects representing the actions to retain
   *                in the pruned graph.
   * @return A new DerivedResolutionGraph containing only the nodes the specified actions
   *         depend on.
   */
  plansys2::DerivedResolutionGraph pruneGraphToActions(
    const std::vector<plansys2::ActionVariant> & actions);

  void appendActions(const std::vector<plansys2::ActionVariant> & actions);
  void appendAction(const plansys2::ActionVariant & action);

  /**
   * @brief Computes the strongly connected components (SCCs) of the graph.
   *
   * This function returns the SCCs of the graph using Tarjan's algorithm.
   * It only includes derived predicates in the SCCs.
   *
   * @return std::vector<std::vector<Derived>> A vector of SCCs, each represented as a
   * vector of Derived predicates.
   */
  std::vector<std::vector<Derived>> computeSCCsTarjanDerivedPredicates() const;

  /**
   * @brief Helper function to compute SCCs
   *
   */
  void strongConnect(
    const Derived & node, int & current_index, std::unordered_map<Derived, int> & index,
    std::unordered_map<Derived, int> & lowlink, std::unordered_set<Derived> & on_stack,
    std::stack<Derived> & stack, std::vector<std::vector<Derived>> & sccs) const;

  const std::unordered_set<NodeVariant> & getNodeOutEdges(const NodeVariant & node)
  {
    return adj_list_[node];
  }
  const std::unordered_set<NodeVariant> & getNodeInEdges(const NodeVariant & node)
  {
    return parent_nodes_[node];
  }
  const std::unordered_set<NodeVariant> & getRoots() const {return roots_;}
  const std::unordered_set<NodeVariant> & getNodes() const {return nodes_;}

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
  std::unordered_set<NodeVariant> getParentNodes(const NodeVariant & node) const;
  std::vector<std::string> getParentNodesNames(const NodeVariant & node) const;

  void exportToDOT(const std::string & filename) const;

  void clear()
  {
    adj_list_.clear();
    parent_nodes_.clear();
    nodes_.clear();
    roots_.clear();
    edge_count_ = 0;
  }

  bool operator==(const DerivedResolutionGraph & other) const
  {
    if (this == &other) {return true;}
    return adj_list_ == other.adj_list_ && parent_nodes_ == other.parent_nodes_ &&
           nodes_ == other.nodes_ && roots_ == other.roots_ &&
           derived_predicates_ == other.derived_predicates_ && actions_ == other.actions_ &&
           node_ids_ == other.node_ids_ && edge_count_ == other.edge_count_;
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

  void invertedDfsHelper(
    const NodeVariant & node, const std::function<void(const NodeVariant &)> & func,
    std::unordered_set<NodeVariant> & visited) const;

  void dfsHelper(
    const NodeVariant & node, const std::function<void(const NodeVariant &)> & func,
    std::unordered_set<NodeVariant> & visited, bool check_dependencies = false) const;

  bool parentsVisited(
    const NodeVariant & node, const std::unordered_set<NodeVariant> & visited) const;

  friend struct std::hash<DerivedResolutionGraph>;
};

}  // namespace plansys2

namespace std
{
template<typename NodeEdgesMap>
std::size_t unordered_nodeedgesmap_hash(const NodeEdgesMap & map)
{
  std::size_t seed = 0;
  for (const auto & [key, neighbors] : map) {
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

#endif  // PLANSYS2_CORE__DERIVEDRESOLUTIONGRAPH_HPP_
