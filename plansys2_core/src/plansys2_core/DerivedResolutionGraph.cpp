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

#include "plansys2_core/DerivedResolutionGraph.hpp"

#include <fstream>

namespace plansys2
{

DerivedResolutionGraph::DerivedResolutionGraph(
  const std::vector<plansys2_msgs::msg::Derived> & derived_predicates)
{
  derived_predicates_ =
    convertVectorToUnorderedSet<plansys2::Derived, plansys2_msgs::msg::Derived>(derived_predicates);
  for (const auto & derived : derived_predicates) {
    addEdgeFromPreconditions(static_cast<plansys2::Derived>(derived), derived.preconditions);
  }
}

DerivedResolutionGraph::DerivedResolutionGraph(
  const std::vector<plansys2::Derived> & derived_predicates)
{
  derived_predicates_ =
    convertVectorToUnorderedSet<plansys2::Derived, plansys2::Derived>(derived_predicates);
  for (const auto & derived : derived_predicates) {
    addEdgeFromPreconditions(derived, derived.preconditions);
  }
}

DerivedResolutionGraph::DerivedResolutionGraph(
  const std::unordered_set<plansys2::Derived> & derived_predicates)
: derived_predicates_(derived_predicates)
{
  for (const auto & derived : derived_predicates) {
    addEdgeFromPreconditions(derived, derived.preconditions);
  }
}

void DerivedResolutionGraph::printGraph() const
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

void DerivedResolutionGraph::printGraphLayers() const
{
  // Step 1: Find root nodes (no incoming edges)
  std::vector<NodeVariant> roots;
  for (const auto & [node, _] : adj_list_) {
    if (parent_nodes_.find(node) == parent_nodes_.end()) {
      roots.push_back(node);
    }
  }

  // Step 2: BFS to assign depth
  std::unordered_map<NodeVariant, int> depth;
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

void DerivedResolutionGraph::addNode(const NodeVariant & node)
{
  if (nodes_.insert(node).second) {  // check if it was actually inserted
    node_ids_[node] = next_id_++;
    if (parent_nodes_.find(node) == parent_nodes_.end()) {
      roots_.insert(node);
    }
  }
}

void DerivedResolutionGraph::addEdge(const NodeVariant & u, const NodeVariant & v)
{
  if (u.isDerived()) {
    u.getDerivedNode().computeNormalizedDerived();
    derived_predicates_.insert(u.getDerivedNode());
  }
  if (v.isDerived()) {
    v.getDerivedNode().computeNormalizedDerived();
    derived_predicates_.insert(v.getDerivedNode());
  }
  if (u.isAction() || u.isDurativeAction()) {
    actions_.insert(u.getActionVariantNode());
  }
  if (v.isAction() || v.isDurativeAction()) {
    actions_.insert(v.getActionVariantNode());
  }

  auto [it, inserted] = adj_list_[u].insert(v);
  if (inserted) {
    ++edge_count_;
  }
  parent_nodes_[v].insert(u);

  addNode(u);
  addNode(v);

  roots_.erase(v);
}

void DerivedResolutionGraph::addEdgeFromPreconditions(
  const NodeVariant & node, const plansys2_msgs::msg::Tree & tree)
{
  for (const auto & tree_node : tree.nodes) {
    switch (tree_node.node_type) {
      case plansys2_msgs::msg::Node::PREDICATE: {
          bool is_derived_predicate = false;
          for (const auto & d : derived_predicates_) {
            if (parser::pddl::checkNodeEquality(tree_node, d.predicate, false)) {
              addEdge(static_cast<const plansys2::Derived &>(d), node);
              is_derived_predicate = true;
            }
          }
          if (!is_derived_predicate) {
            addEdge(static_cast<const plansys2::Predicate &>(tree_node), node);
          }
          break;
        }
      case plansys2_msgs::msg::Node::FUNCTION:
        addEdge(static_cast<const plansys2::Function &>(tree_node), node);
        break;
      default:
        break;
    }
  }
}

// DFS from a given start node
void DerivedResolutionGraph::depthFirstTraverse(
  const NodeVariant & start, const std::function<void(const NodeVariant &)> & func,
  std::unordered_set<NodeVariant> & visited, bool check_dependencies) const
{
  dfsHelper(start, func, visited, check_dependencies);
}

void DerivedResolutionGraph::depthFirstTraverse(
  const NodeVariant & start, const std::function<void(const NodeVariant &)> & func,
  bool check_dependencies) const
{
  std::unordered_set<NodeVariant> visited;
  dfsHelper(start, func, visited, check_dependencies);
}

void DerivedResolutionGraph::depthFirstTraverseFromNodes(
  const std::function<void(const NodeVariant &)> & func, bool check_dependencies,
  const std::vector<NodeVariant> & start_nodes) const
{
  std::unordered_set<NodeVariant> visited;
  std::vector<NodeVariant> stack =
    start_nodes.empty() ? std::vector<NodeVariant>(getRoots().begin(), getRoots().end()) :
    start_nodes;

  while (!stack.empty()) {
    NodeVariant node = stack.back();
    stack.pop_back();

    if (visited.count(node)) {continue;}
    if (check_dependencies && !parentsVisited(node, visited)) {
      continue;  // skip nodes whose dependencies aren't met
    }
    visited.insert(node);
    func(node);

    auto it = adj_list_.find(node);
    if (it != adj_list_.end()) {
      for (const auto & neighbor : it->second) {
        if (!visited.count(neighbor)) {stack.push_back(neighbor);}
      }
    }
  }
}

void DerivedResolutionGraph::depthFirstTraverseAll(
  const std::function<void(const NodeVariant &)> & func, bool check_dependencies) const
{
  std::vector<NodeVariant> roots(getRoots().begin(), getRoots().end());
  depthFirstTraverseFromNodes(func, check_dependencies, roots);
}

std::vector<plansys2::Derived> DerivedResolutionGraph::getDerivedPredicatesDepthFirst(
  const std::vector<NodeVariant> & start_nodes) const
{
  std::vector<plansys2::Derived> all_nodes;
  auto traversal = [&all_nodes](const plansys2::NodeVariant & node) {
      if (node.isDerived()) {
        all_nodes.push_back(node.getDerivedNode());
      }
    };

  if (start_nodes.empty()) {
    this->depthFirstTraverseAll(traversal, true);
  } else {
    auto sub_graph = getSubGraphFromNodes(start_nodes);
    sub_graph.depthFirstTraverseFromNodes(traversal, true, start_nodes);
  }
  return all_nodes;
}

std::deque<plansys2::Derived> DerivedResolutionGraph::getDerivedPredicatesFromActions(
  const std::vector<plansys2::ActionVariant> & actions) const
{
  std::deque<plansys2::Derived> derived_predicates;
  std::unordered_set<plansys2::NodeVariant> visited;
  auto func = [&derived_predicates](const plansys2::NodeVariant & node) {
      if (node.isDerived()) {
        derived_predicates.push_front(node.getDerivedNode());
      }
    };
  for (const auto & action : actions) {
    backtrackTraverse(action, visited, func);
  }
  return derived_predicates;
}

void DerivedResolutionGraph::backtrackTraverse(
  const NodeVariant & start, const std::function<void(const NodeVariant &)> & func) const
{
  std::unordered_set<NodeVariant> visited;
  backtrackTraverse(start, visited, func);
}

void DerivedResolutionGraph::backtrackTraverse(
  const NodeVariant & node, std::unordered_set<NodeVariant> & visited,
  const std::function<void(const NodeVariant &)> & func) const
{
  std::vector<NodeVariant> stack;
  stack.push_back(node);

  while (!stack.empty()) {
    NodeVariant node = stack.back();
    stack.pop_back();

    if (visited.count(node)) {continue;}
    visited.insert(node);
    func(node);

    auto it = parent_nodes_.find(node);
    if (it != parent_nodes_.end()) {
      for (const auto & neighbor : it->second) {
        if (!visited.count(neighbor)) {stack.push_back(neighbor);}
      }
    }
  }
}

DerivedResolutionGraph DerivedResolutionGraph::getSubGraphFromNodes(
  const std::vector<NodeVariant> & nodes) const
{
  DerivedResolutionGraph sub_graph;
  std::unordered_set<NodeVariant> visited;
  std::vector<NodeVariant> stack = nodes;
  while (!stack.empty()) {
    NodeVariant node = stack.back();
    stack.pop_back();

    if (visited.count(node)) {continue;}
    visited.insert(node);

    auto it = adj_list_.find(node);
    if (it != adj_list_.end()) {
      for (const auto & neighbor : it->second) {
        sub_graph.addEdge(node, neighbor);
        if (!visited.count(neighbor)) {
          stack.push_back(neighbor);
        }
      }
    } else {
      sub_graph.addNode(node);
    }
  }
  return sub_graph;
}

plansys2::DerivedResolutionGraph DerivedResolutionGraph::pruneGraphToActions(
  const std::vector<plansys2::ActionVariant> & actions)
{
  auto new_graph = DerivedResolutionGraph();
  auto func_new_graph = [this, &new_graph](const plansys2::NodeVariant & node) {
      for (const auto & parent_node : this->getNodeInEdges(node)) {
        new_graph.addEdge(parent_node, node);
      }
    };

  std::unordered_set<NodeVariant> visited;
  for (const auto & action : actions) {
    backtrackTraverse(action, visited, func_new_graph);
  }
  return new_graph;
}

void DerivedResolutionGraph::appendActions(const std::vector<plansys2::ActionVariant> & actions)
{
  for (const auto & action : actions) {
    appendAction(action);
  }
}

void DerivedResolutionGraph::appendAction(const plansys2::ActionVariant & action)
{
  addEdgeFromPreconditions(action, action.get_overall_requirements());
  if (action.is_durative_action()) {
    addEdgeFromPreconditions(action, action.get_at_start_requirements());
    addEdgeFromPreconditions(action, action.get_at_end_requirements());
  }
}

std::vector<std::vector<Derived>> DerivedResolutionGraph::computeSCCsTarjanDerivedPredicates() const
{
  std::vector<std::vector<Derived>> sccs;
  std::unordered_map<Derived, int> index;
  std::unordered_map<Derived, int> lowlink;
  std::unordered_set<Derived> on_stack;
  std::stack<Derived> stack;
  int current_index = 0;

  for (const auto & node : this->derived_predicates_) {
    if (index.find(node) == index.end()) {
      strongConnect(node, current_index, index, lowlink, on_stack, stack, sccs);
    }
  }
  return sccs;
}

void DerivedResolutionGraph::strongConnect(
  const Derived & node, int & current_index, std::unordered_map<Derived, int> & index,
  std::unordered_map<Derived, int> & lowlink, std::unordered_set<Derived> & on_stack,
  std::stack<Derived> & stack, std::vector<std::vector<Derived>> & sccs) const
{
  index[node] = current_index;
  lowlink[node] = current_index;
  ++current_index;
  stack.push(node);
  on_stack.insert(node);

  auto it = adj_list_.find(node);
  if (it != adj_list_.end()) {
    for (const auto & child_node : it->second) {
      if (!child_node.isDerived()) {continue;}
      Derived child_derived = child_node.getDerivedNode();
      if (index.find(child_derived) == index.end()) {
        strongConnect(child_derived, current_index, index, lowlink, on_stack, stack, sccs);
        lowlink[node] = std::min(lowlink[node], lowlink[child_derived]);
      } else if (on_stack.count(child_derived)) {
        lowlink[node] = std::min(lowlink[node], index[child_derived]);
      }
    }
  }

  if (lowlink[node] == index[node]) {
    std::vector<Derived> new_scc;
    while (true) {
      Derived w = stack.top();
      stack.pop();
      on_stack.erase(w);
      new_scc.push_back(w);
      if (w == node) {break;}
    }
    sccs.push_back(std::move(new_scc));
  }
}

std::vector<std::string> DerivedResolutionGraph::getRootsNames() const
{
  std::vector<std::string> root_names;
  for (const auto & root : roots_) {
    root_names.push_back(root.getNodeName());
  }
  return root_names;
}

std::vector<std::string> DerivedResolutionGraph::getNodesNames() const
{
  std::vector<std::string> nodes_names;
  for (const auto & node : nodes_) {
    nodes_names.push_back(node.getNodeName());
  }
  return nodes_names;
}

std::vector<std::string> DerivedResolutionGraph::getPredicatesNames() const
{
  std::vector<std::string> predicates_names;
  for (const auto & node : nodes_) {
    if (node.isPredicate()) {
      predicates_names.push_back(node.getNodeName());
    }
  }
  return predicates_names;
}

std::vector<plansys2::Predicate> DerivedResolutionGraph::getPredicates() const
{
  std::vector<plansys2::Predicate> predicates_;
  for (const auto & node : nodes_) {
    if (node.isPredicate()) {
      predicates_.push_back(node.getPredicateNode());
    }
  }
  return predicates_;
}

std::vector<std::string> DerivedResolutionGraph::getFunctionsNames() const
{
  std::vector<std::string> functions_;
  for (const auto & node : nodes_) {
    if (node.isFunction()) {
      functions_.push_back(node.getNodeName());
    }
  }
  return functions_;
}

std::vector<plansys2::Function> DerivedResolutionGraph::getFunctions() const
{
  std::vector<plansys2::Function> functions_;
  for (const auto & node : nodes_) {
    if (node.isFunction()) {
      functions_.push_back(node.getFunctionNode());
    }
  }
  return functions_;
}

std::vector<std::string> DerivedResolutionGraph::getDerivedPredicatesNames() const
{
  std::vector<std::string> predicates_names;
  for (const auto & node : nodes_) {
    if (node.isDerived()) {
      predicates_names.push_back(node.getNodeName());
    }
  }
  return predicates_names;
}

std::unordered_set<NodeVariant> DerivedResolutionGraph::getParentNodes(
  const NodeVariant & node) const
{
  auto it = parent_nodes_.find(node);
  if (it != parent_nodes_.end()) {return it->second;}
  return {};
}

std::vector<std::string> DerivedResolutionGraph::getParentNodesNames(const NodeVariant & node) const
{
  std::vector<std::string> parent_nodes_names;
  auto it = parent_nodes_.find(node);
  if (it != parent_nodes_.end()) {
    for (const auto & n : it->second) {
      parent_nodes_names.push_back(n.getNodeName());
    }
  }
  return parent_nodes_names;
}

void DerivedResolutionGraph::invertedDfsHelper(
  const NodeVariant & node, const std::function<void(const NodeVariant &)> & func,
  std::unordered_set<NodeVariant> & visited) const
{
  if (visited.count(node)) {
    return;  // Skip this node if already visited
  }

  visited.insert(node);
  func(node);

  auto it = parent_nodes_.find(node);
  if (it != parent_nodes_.end()) {
    for (const auto & parent : it->second) {
      if (!visited.count(parent)) {
        visited.insert(parent);
        invertedDfsHelper(parent, func, visited);
      }
    }
  }
}

void DerivedResolutionGraph::dfsHelper(
  const NodeVariant & node, const std::function<void(const NodeVariant &)> & func,
  std::unordered_set<NodeVariant> & visited, bool check_dependencies) const
{
  if (visited.count(node) || (check_dependencies && !parentsVisited(node, visited))) {
    return;  // Skip this node if dependencies are not satisfied
  }
  visited.insert(node);
  func(node);
  auto it = adj_list_.find(node);
  if (it != adj_list_.end()) {
    for (const auto & neighbor : it->second) {
      dfsHelper(neighbor, func, visited, check_dependencies);
    }
  }
}

bool DerivedResolutionGraph::parentsVisited(
  const NodeVariant & node, const std::unordered_set<NodeVariant> & visited) const
{
  auto it = parent_nodes_.find(node);
  if (it == parent_nodes_.end()) {
    return true;                                // No parents
  }
  for (const auto & p : it->second) {
    if (!visited.count(p)) {return false;}
  }
  return true;
}

void DerivedResolutionGraph::exportToDOT(const std::string & filename) const
{
  std::ofstream file(filename);
  if (!file.is_open()) {return;}

  file << "digraph G {\n";
  for (const auto & node : nodes_) {
    size_t id = node_ids_.at(node);
    file << "  n" << id << " [label=\"" << node << "\"];\n";
  }
  for (const auto & [from, tos] : adj_list_) {
    size_t from_id = node_ids_.at(from);
    for (const auto & to : tos) {
      size_t to_id = node_ids_.at(to);
      file << "  n" << from_id << " -> n" << to_id << ";\n";
    }
  }
  file << "}\n";
}

}  // namespace plansys2
