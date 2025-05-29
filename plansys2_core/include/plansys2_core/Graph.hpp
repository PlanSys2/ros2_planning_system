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

#ifndef PLANSYS2_CORE__GRAPH_HPP_
#define PLANSYS2_CORE__GRAPH_HPP_

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

#include "plansys2_core/Action.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

/**
 * @class NodeVariant
 * @brief A wrapper class for representing different types of nodes in the planning system.
 *
 * NodeVariant encapsulates a variant type that can hold one of several node types used in the planning system,
 * including Predicate, Function, Derived, and ActionVariant. It provides utility methods for type checking,
 * accessing the underlying node, and retrieving node-specific information such as name and type.
 *
 * The class supports hashing, equality comparison, and printing node information for debugging purposes.
 * 
 * This class uses a custom hash function and '==' operators designed specifically for enabling the 
 * resolution of derived predicates in the correct order, which might not be suitable for other uses. 
 * For example (check the unit tests for more examples): 
 *  - (predA ?a) == (predA ?x)
 *
 * @note The underlying node is stored as a shared pointer to allow for efficient copying and management.
 *
 * @author
 * @date
 */
class NodeVariant
{
public:
  using NodeType = std::variant<
    plansys2::Predicate, plansys2::Function, plansys2::Derived, plansys2::ActionVariant>;

  template<typename NodeT>
  NodeVariant(NodeT node)  // NOLINT(runtime/explicit)
  : node_(std::make_shared<NodeType>(node))
  {
  }

  size_t hash() const
  {
    return std::visit(
      [](auto && arg) {return std::hash<std::decay_t<decltype(arg)>>{}(arg);}, *node_);
  }

  bool operator==(const NodeVariant& other) const
  {
    // Check type first
    if (this->getNodeType() != other.getNodeType())
      return false;

    // Predicate case
    if (this->isPredicate()) {
      return parser::pddl::checkNodeEquality(this->getPredicateNode(), other.getPredicateNode(), false);
    }

    // Function case
    if (this->isFunction()) {
      return parser::pddl::checkNodeEquality(this->getFunctionNode(), other.getFunctionNode(), false);
    }

    return *node_ == *other.node_;
  }

  bool operator!=(const NodeVariant& other) const {
    return !(*this == other);
  }

  const NodeType & getNode() const {return *node_;}

  std::string getNodeName() const
  {
    std::string node_name;
    if (std::holds_alternative<plansys2::Predicate>(*node_)) {
      node_name = std::get<plansys2::Predicate>(*node_).name;
    } else if (std::holds_alternative<plansys2::Function>(*node_)) {
      node_name = std::get<plansys2::Function>(*node_).name;
    } else if (std::holds_alternative<plansys2::Derived>(*node_)) {
      node_name = std::get<plansys2::Derived>(*node_).predicate.name;
    } else if (std::holds_alternative<plansys2::ActionVariant>(*node_)) {
      node_name = std::get<plansys2::ActionVariant>(*node_).get_action_name();
    }

    return node_name;
  }

  bool isPredicate() const {return std::holds_alternative<plansys2::Predicate>(*node_);}

  bool isFunction() const {return std::holds_alternative<plansys2::Function>(*node_);}

  bool isDerived() const {return std::holds_alternative<plansys2::Derived>(*node_);}

  bool isAction() const
  {
    return std::holds_alternative<plansys2::ActionVariant>(*node_) &&
           std::get<plansys2::ActionVariant>(*node_).is_action();
  }
  
  bool isDurativeAction() const
  {
    return std::holds_alternative<plansys2::ActionVariant>(*node_) &&
           std::get<plansys2::ActionVariant>(*node_).is_durative_action();
  }

  // plansys2::Predicate & getPredicate() const {return std::get<plansys2::Predicate>(*node_);}
  plansys2::Function & getFunctionNode() const {return std::get<plansys2::Function>(*node_);}
  plansys2::Predicate getPredicateNode() const {return std::get<plansys2::Predicate>(*node_);}
  plansys2::Derived getDerivedNode() const {return std::get<plansys2::Derived>(*node_);}

  auto & getDerivedPreconditions() const
  {
    return std::get<plansys2::Derived>(*node_).preconditions;
  }

  auto & getDerivedPredicate() const {return std::get<plansys2::Derived>(*node_).predicate;}

  std::string getNodeType() const
  {
    if (std::holds_alternative<plansys2::Predicate>(*node_)) {
      return "predicate";
    } else if (std::holds_alternative<plansys2::Function>(*node_)) {
      return "function";
    } else if (std::holds_alternative<plansys2::Derived>(*node_)) {
      return "derived";
    } else if (std::holds_alternative<plansys2::ActionVariant>(*node_)) {
      return std::get<plansys2::ActionVariant>(*node_).is_action() ? "action" : "durative action";
    }
    return "";
  }

  void printNode() const
  {
    if(isDerived()) {
      std::cout << "  Derived Predicate: " << getDerivedPredicate().name;
      for (const auto & param : getDerivedPredicate().parameters) {
        std::cout << " " << param.name;
      }
      std::cout << "\n";
    }
    if(isPredicate()) {
      std::cout << "  Predicate: " << getPredicateNode().name;
      for (const auto & param : getPredicateNode().parameters) {
        std::cout << " " << param.name;
      }
      std::cout << "\n";
    }
    if(isFunction()) {
      std::cout << "  Function: " << getFunctionNode().name;
      for (const auto & param : getFunctionNode().parameters) {
        std::cout << " " << param.name;
      }
      std::cout << "\n";
    }
    if(isAction()) {
      std::cout << "  Action: " << getNodeName() << "\n";
    }
    if(isDurativeAction()) {
      std::cout << "  Durative Action: " << getNodeName() << "\n";
    }
  }

private:
  std::shared_ptr<NodeType> node_;

  friend struct std::hash<NodeVariant>;
};

inline bool operator==(const NodeVariant & lhs, const plansys2_msgs::msg::Node & rhs)
{
  if (lhs.isPredicate()) {
    return lhs.getPredicateNode() == static_cast<plansys2::Predicate>(rhs);
  } else if (lhs.isFunction()) {
    return lhs.getFunctionNode() == static_cast<plansys2::Function>(rhs);
  }
  return false;
}

}  // namespace plansys2

namespace std
{

inline std::size_t hash_node_variant(const plansys2_msgs::msg::Node & node)
{
  std::size_t seed = 0;

  hash_combine(seed, node.name);
  hash_combine(seed, node.node_type);
  hash_combine(seed, node.parameters.size());

  return seed;
}

template<>
struct hash<plansys2::NodeVariant>
{
  std::size_t operator()(const plansys2::NodeVariant & nv) const noexcept 
  {
    // return nv.hash();
    if (nv.isPredicate())
    {
      return hash_node_variant(nv.getPredicateNode());
    } else if(nv.isFunction())
    {
      return hash_node_variant(nv.getFunctionNode());
    } 
    return nv.hash();
  }
};
}  // namespace std

namespace plansys2
{
class Graph
{
public:
  Graph()
  : edge_count_(0) {}

  void printGraph() const 
  {
    std::cout << "Graph structure:\n";
    for (const auto & [node, edges] : adj_list_) {
      node.printNode();
      for (const auto & child : edges) {
        std::cout << "    -> "; 
        child.printNode();
      }
    }
  }

  void printGraphLayers() const
  {
    // Step 1: Find root nodes (no incoming edges)
    std::vector<NodeVariant> roots;
    for (const auto & [node, _] : adj_list_) {
      if (in_nodes_.find(node) == in_nodes_.end()) {
        roots.push_back(node);
      }
    }

    // Step 2: BFS to assign depth
    std::unordered_map<NodeVariant, int> depth;
    // std::map<int, std::vector<NodeVariant>> layers;  // keep this if you want ordered layers
    std::queue<NodeVariant> q;
    for (const auto & root : roots) {
      depth.insert({root, 0});
      q.push(root);
    }

    while (!q.empty()) {
      NodeVariant current = q.front();
      q.pop();

      int current_depth = depth[current];
      auto it = adj_list_.find(current);
      if (it != adj_list_.end()) {
        for (const auto & child : it->second) {
          if (depth.find(child) == depth.end()) {
            // depth[child] = current_depth + 1;
            depth.insert({child, current_depth + 1});
            q.push(child);
          }
        }
      }
    }

    // Step 3: Group by layer
    std::map<int, std::vector<NodeVariant>> layers;
    for (const auto & [node, d] : depth) {
      layers[d].push_back(node);
    }

    // Step 4: Print layers
    std::cout << "\n=== Graph Layers ===\n";
    for (const auto & [d, nodes] : layers) {
      std::cout << "Layer " << d << ":\n";
      for (const auto & node : nodes) {
        std::cout << "  - " << node.getNodeName() << "\n";
      }
    }
    std::cout << "====================\n";
  }

  void addNode(const NodeVariant & node)
  {
    nodes_.insert(node);
    if(in_nodes_.find(node) == in_nodes_.end()) {
      roots_.insert(node);
    }
  }

  void addEdge(const NodeVariant & u, const NodeVariant & v)
  {
    auto [it, inserted] = adj_list_[u].insert(v);
    if (inserted) {
      ++edge_count_;
    }
    in_nodes_[v].insert(u);
    
    nodes_.insert(u);
    nodes_.insert(v);
    
    if(in_nodes_.find(u) == in_nodes_.end()) {
      roots_.insert(u);
    }
    roots_.erase(v); 
  }

  // DFS from a given start node
  void depthFirstTraverse(const NodeVariant& start, 
    const std::function<void(const NodeVariant&)>& func,
    std::unordered_set<NodeVariant>& visited,
    bool check_dependencies = false) const 
  {
    dfsHelper(start, func, visited, check_dependencies);
  }

  void depthFirstTraverse(const NodeVariant& start, 
    const std::function<void(const NodeVariant&)>& func,
    bool check_dependencies = false) const 
  {
    std::unordered_set<NodeVariant> visited;
    dfsHelper(start, func, visited, check_dependencies);
  }

  template<typename Func>
  void depthFirstTraverseFromNodes(
    Func && func, 
    bool check_dependencies = false,
    const std::vector<NodeVariant>& start_nodes = {}) const 
  {
    std::unordered_set<NodeVariant> visited;
    std::vector<NodeVariant> stack = start_nodes.empty() 
      ? std::vector<NodeVariant>(getRoots().begin(), getRoots().end()) : start_nodes;

    while (!stack.empty()) {
      NodeVariant node = stack.back();
      stack.pop_back();

      if (visited.count(node)) continue;
      if (check_dependencies && !parentsVisited(node, visited)) {
        continue; // skip nodes whose dependencies aren't met
      }
      visited.insert(node);
      func(node);

      auto it = adj_list_.find(node);
      if (it != adj_list_.end()) {
        for (const auto& neighbor : it->second) {
          if (!visited.count(neighbor))
              stack.push_back(neighbor);
        }
      }
    }
  }

  void depthFirstTraverseAll(
    const std::function<void(const NodeVariant&)>& func,
    bool check_dependencies = false) const
  {
    std::vector<NodeVariant> roots(getRoots().begin(), getRoots().end());
    depthFirstTraverseFromNodes(func, check_dependencies, roots);
  }

  void backtrackTraverse(const NodeVariant & start, const std::function<void(const NodeVariant&)>& func) const
  {
    std::unordered_set<NodeVariant> visited;
    backtrackTraverse(start, visited, func);
  }

  void backtrackTraverse(
    const NodeVariant & node, std::unordered_set<NodeVariant> & visited, 
    const std::function<void(const NodeVariant&)>& func) const
  {
    std::vector<NodeVariant> stack;
    stack.push_back(node);

    while (!stack.empty()) {
      NodeVariant node = stack.back();
      stack.pop_back();

      if (visited.count(node)) continue;
      visited.insert(node);
      func(node);

      auto it = in_nodes_.find(node);
      if (it != in_nodes_.end()) {
          for (const auto& neighbor : it->second) {
              if (!visited.count(neighbor))
                stack.push_back(neighbor);
          }
      }
    }
  }

  Graph getSubGraphFromNodes(const std::vector<NodeVariant> & nodes) const
  {
    Graph sub_graph;
    std::unordered_set<NodeVariant> visited;
    std::vector<NodeVariant> stack = nodes;
    while (!stack.empty()) {
      NodeVariant node = stack.back();
      stack.pop_back();

      if (visited.count(node)) continue;
      visited.insert(node);

      auto it = adj_list_.find(node);
      if (it != adj_list_.end()) {
        for (const auto& neighbor : it->second) {
          sub_graph.addEdge(node, neighbor);
          if (!visited.count(neighbor))
          {
            stack.push_back(neighbor);
          }
        }
      } else {
        sub_graph.addNode(node);
      }
    }
    return sub_graph;
  }

  const std::unordered_set<NodeVariant>& getNodeOutEdges(const NodeVariant & node) {return adj_list_[node];}
  const std::unordered_set<NodeVariant>& getNodeInEdges(const NodeVariant & node) {return in_nodes_[node];}
  const std::unordered_set<NodeVariant>& getRoots() const { return roots_; }
  const std::unordered_set<NodeVariant>& getNodes() const { return nodes_; }
  
  auto getEdgeNumber() const {return edge_count_;}
  auto getNodeNumber() const {return nodes_.size();}
  auto getRootNumber() const {return roots_.size();} 

  auto getRootsNames()
  {
    std::vector<std::string> root_names;
    for (const auto & root : roots_) {
      root_names.push_back(root.getNodeName());
    }
    return root_names;
  }

  auto getNodesNames()
  {
    std::vector<std::string> nodes_names;
    for (const auto & node : nodes_) {
      nodes_names.push_back(node.getNodeName());
    }
    return nodes_names;
  }

  void clear() {adj_list_.clear(); in_nodes_.clear(); nodes_.clear(); roots_.clear(); edge_count_ = 0;}

  bool operator==(const Graph & graph) const 
  {
    return this->adj_list_ == graph.adj_list_ && this->in_nodes_ == graph.in_nodes_ &&
      this->nodes_ == graph.nodes_ && this->edge_count_ == graph.edge_count_;
  }

  using NodeEdgesMap = std::unordered_map<NodeVariant, std::unordered_set<NodeVariant>>;

private:
  NodeEdgesMap adj_list_;
  NodeEdgesMap in_nodes_;
  std::unordered_set<NodeVariant> nodes_;
  std::unordered_set<NodeVariant> roots_;
  size_t edge_count_;

  void invertedDfsHelper(const NodeVariant& node, 
    const std::function<void(const NodeVariant&)>& func,
    std::unordered_set<NodeVariant>& visited) const
  {
    if (visited.count(node)) 
    {
      return;  // Skip this node if already visited
    }
    
    visited.insert(node);
    func(node);

    auto it = in_nodes_.find(node);
    if (it != in_nodes_.end()) {
      for (const auto& parent : it->second) {
        if (!visited.count(parent)) {
          visited.insert(parent);
          invertedDfsHelper(parent, func, visited);
        }
      }
    }

  }

  template<typename Func>
  void dfsHelper(const NodeVariant& node, 
    Func && func,
    std::unordered_set<NodeVariant>& visited,
    bool check_dependencies = false) const 
  {
    if (visited.count(node) || (check_dependencies && !parentsVisited(node, visited))) 
    {
      return;  // Skip this node if dependencies are not satisfied
    }
    visited.insert(node);
    // func(node);
    std::forward<Func>(func)(node);
    auto it = adj_list_.find(node);
    if (it != adj_list_.end()) {
      for (const auto& neighbor : it->second) {
        dfsHelper(neighbor, func, visited, check_dependencies);
      }
    }
  }

  bool parentsVisited(const NodeVariant& node, const std::unordered_set<NodeVariant>& visited) const 
  {
    auto it = in_nodes_.find(node);
    if (it == in_nodes_.end()) return true; // No parents
    for (const auto& p : it->second) {
        if (!visited.count(p)) return false;
    }
    return true;
  }

  friend struct std::hash<Graph>;
};

class DerivedGraph : public plansys2::Graph
{
public:
  DerivedGraph()
  : Graph() {}

  DerivedGraph(const std::vector<plansys2_msgs::msg::Derived> & derived_predicates);  // NOLINT

  DerivedGraph(const std::vector<plansys2::Derived> & derived_predicates);  // NOLINT

  DerivedGraph(const std::unordered_set<plansys2::Derived> & derived_predicates);  // NOLINT

  auto & getDerivedPredicates() const {return derived_predicates_;}

  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst(
    const std::vector<NodeVariant>& start_nodes = {}) const;

  std::deque<plansys2::Derived> getDerivedPredicatesFromActions(
    const std::vector<plansys2::ActionVariant> & actions) const;

  plansys2::DerivedGraph pruneGraphToActions(const std::vector<plansys2::ActionVariant> & actions);

  void appendActions(const std::vector<plansys2::ActionVariant> & actions);
  void appendAction(const plansys2::ActionVariant & action);

  void addEdge(const NodeVariant & u, const NodeVariant & v);
  void addEdgeFromPreconditions(const NodeVariant & node, const plansys2_msgs::msg::Tree & tree);

private:
  std::unordered_set<plansys2::Derived> derived_predicates_;
};

}  // namespace plansys2

namespace std
{
template<>
struct hash<plansys2::Graph>
{
  std::size_t operator()(const plansys2::Graph & graph) const noexcept
  {
    std::size_t seed = 0;
    for (const auto & [key, neighbors] : graph.adj_list_) {
      hash_combine(seed, key);
      for (const auto & neighbor : neighbors) {
        hash_combine(seed, neighbor);
      }
    }
    return seed;
  }
};

template<>
struct hash<plansys2::DerivedGraph>
{
  std::size_t operator()(const plansys2::DerivedGraph & graph) const noexcept
  {
    return std::hash<plansys2::Graph>{}(graph);
  }
};
}  // namespace std

#endif  // PLANSYS2_CORE__GRAPH_HPP_
