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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <utility>
#include <tuple>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

/**
 * @brief Node in an action graph representing a single action from the plan.
 *
 * Contains information about a single action, its dependencies, and the state
 * of the world before and after its execution.
 */
struct ActionNode
{
  using Ptr = std::shared_ptr<ActionNode>;
  static Ptr make_shared() {return std::make_shared<ActionNode>();}

  ActionStamped action;
  int node_num;
  int level_num;

  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;

  std::list<ActionNode::Ptr> in_arcs;
  std::list<ActionNode::Ptr> out_arcs;
};

/**
 * @brief Graph structure representing a plan as a directed acyclic graph of actions.
 *
 * Contains root nodes (actions that can start immediately) and levels (actions
 * organized by their start time).
 */
struct ActionGraph
{
  using Ptr = std::shared_ptr<ActionGraph>;
  static Ptr make_shared() {return std::make_shared<ActionGraph>();}

  std::list<ActionNode::Ptr> roots;
  std::map<float, std::list<ActionNode::Ptr>> levels;
};

/**
 * @brief Simple implementation of a behavior tree builder.
 *
 * Converts a sequential plan into a behavior tree by analyzing action dependencies
 * based on their preconditions and effects, constructing a graph, and then
 * transforming the graph into a behavior tree.
 */
class SimpleBTBuilder : public BTBuilder
{
public:
  /**
   * @brief Constructor for SimpleBTBuilder.
   *
   * Initializes domain and problem clients.
   */
  SimpleBTBuilder();

  /**
   * @brief Initializes the builder with behavior tree templates.
   *
   * @param[in] bt_action_1 XML template for regular actions, default empty.
   * @param[in] bt_action_2 XML template for durative actions, default empty.
   * @param[in] precision Precision for time calculations, default 3.
   */
  void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3);

  /**
   * @brief Generates a behavior tree XML from a plan.
   *
   * Analyzes the plan, constructs an action graph based on dependencies,
   * and converts the graph into a behavior tree XML.
   *
   * @param[in] current_plan The plan to transform into a behavior tree.
   * @return std::string containing the behavior tree XML.
   */
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);

  /**
   * @brief Not implemented for SimpleBTBuilder.
   *
   * @return nullptr as this function is not implemented.
   */
  Graph::Ptr get_graph() {return nullptr;}

  /**
   * @brief Not implemented for SimpleBTBuilder.
   *
   * @param[in] graph The graph to propagate constraints through.
   * @return Always returns true as this function is not implemented.
   */
  bool propagate(Graph::Ptr) {return true;}

  /**
   * @brief Generates a DOT graph representation of the action graph for visualization.
   *
   * @param[in] action_map Map of action IDs to execution information.
   * @param[in] enable_legend Whether to include a legend in the graph, default false.
   * @param[in] enable_print_graph Whether to print the graph to console, default false.
   * @return std::string containing the DOT graph representation.
   */
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false,
    bool enable_print_graph = false);

protected:
  /**
   * @brief Constructs an action graph from a plan.
   *
   * Analyzes action dependencies based on preconditions and effects
   * to build a directed acyclic graph.
   *
   * @param[in] current_plan The plan to analyze.
   * @return Shared pointer to the constructed action graph.
   */
  ActionGraph::Ptr get_graph(const plansys2_msgs::msg::Plan & current_plan);

  /**
   * @brief Extracts action information from a plan.
   *
   * @param[in] plan The plan to extract actions from.
   * @return std::vector<ActionStamped> Vector of ActionStamped objects representing the plan actions.
   */
  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan);

  /**
   * @brief Removes redundant incoming edges to a node.
   *
   * Prunes the graph by removing redundant dependencies.
   *
   * @param[in] new_node The node being added to the graph.
   * @param[in] node_satisfy A node that satisfies a requirement of new_node.
   */
  void prune_backwards(ActionNode::Ptr new_node, ActionNode::Ptr node_satisfy);

  /**
   * @brief Removes redundant outgoing edges from a node.
   *
   * @param[in] current The node to prune outgoing edges from.
   * @param[in,out] used_nodes List of nodes already processed.
   */
  void prune_forward(ActionNode::Ptr current, std::list<ActionNode::Ptr> & used_nodes);

  /**
   * @brief Computes the state of the world at a node.
   *
   * Recursively applies effects of predecessor actions to determine
   * the state at the current node.
   *
   * @param[in] node The node to compute state for.
   * @param[in,out] used_nodes List of nodes already processed.
   * @param[out] predicates Resulting predicates at this node.
   * @param[out] functions Resulting functions at this node.
   */
  void get_state(
    const ActionNode::Ptr & node,
    std::list<ActionNode::Ptr> & used_nodes,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;

  /**
   * @brief Checks if an action is executable in a given state.
   *
   * @param[in] action The action to check.
   * @param[in] predicates Current predicates.
   * @param[in] functions Current functions.
   * @return True if the action's requirements are satisfied, false otherwise.
   */
  bool is_action_executable(
    const ActionStamped & action,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;

  /**
   * @brief Finds root actions that can be executed immediately.
   *
   * @param[in,out] action_sequence Actions to check, executable ones will be removed.
   * @param[in] predicates Current predicates.
   * @param[in] functions Current functions.
   * @param[in,out] node_counter Counter for assigning unique node IDs.
   * @return std::list<ActionNode::Ptr> List of nodes representing executable actions.
   */
  std::list<ActionNode::Ptr> get_roots(
    std::vector<plansys2::ActionStamped> & action_sequence,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions,
    int & node_counter);

  /**
   * @brief Finds a node that satisfies a given requirement.
   *
   * @param[in] requirement The requirement to satisfy.
   * @param[in] graph The action graph to search.
   * @param[in] current The current node being processed.
   * @return A node that satisfies the requirement, or nullptr if none found.
   */
  ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    const ActionGraph::Ptr & graph,
    const ActionNode::Ptr & current);

  /**
   * @brief Finds a node that satisfies a given requirement within a subtree.
   *
   * @param[in] requirement The requirement to satisfy.
   * @param[in] node The root of the subtree to search.
   * @param[in] current The current node being processed.
   * @return A node that satisfies the requirement, or nullptr if none found.
   */
  ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    const ActionNode::Ptr & node,
    const ActionNode::Ptr & current);

  /**
   * @brief Finds nodes whose execution would be contradicted by the current node.
   *
   * @param[in] graph The action graph to search.
   * @param[in] current The current node being processed.
   * @return List of nodes that would be contradicted.
   */
  std::list<ActionNode::Ptr> get_node_contradict(
    const ActionGraph::Ptr & graph,
    const ActionNode::Ptr & current);

  /**
   * @brief Recursively finds nodes contradicted by the current node within a subtree.
   *
   * @param[in] node The root of the subtree to search.
   * @param[in] current The current node being processed.
   * @param[in,out] parents List to collect contradicted nodes.
   */
  void get_node_contradict(
    const ActionNode::Ptr & node,
    const ActionNode::Ptr & current,
    std::list<ActionNode::Ptr> & parents);

  /**
   * @brief Removes requirements that are already satisfied in the current state.
   *
   * @param[in,out] requirements List of requirements to filter.
   * @param[in] predicates Current predicates.
   * @param[in] functions Current functions.
   */
  void remove_existing_requirements(
    std::vector<plansys2_msgs::msg::Tree> & requirements,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;

  /**
   * @brief Checks if an action can be executed in parallel with a set of other actions.
   *
   * @param[in] action The action to check.
   * @param[in] predicates Current predicates.
   * @param[in] functions Current functions.
   * @param[in] ret List of actions to check parallelization with.
   * @return True if the action can be executed in parallel, false otherwise.
   */
  bool is_parallelizable(
    const plansys2::ActionStamped & action,
    const std::vector<plansys2::Predicate> & predicates,
    const std::vector<plansys2::Function> & functions,
    const std::list<ActionNode::Ptr> & ret) const;

  /**
   * @brief Generates the behavior tree XML for a node and its descendants.
   *
   * @param[in] node The node to process.
   * @param[in,out] used_nodes List of nodes already processed.
   * @param[in] level Indentation level.
   * @return std::string containing the behavior tree XML.
   */
  std::string get_flow_tree(
    ActionNode::Ptr node,
    std::list<std::string> & used_nodes,
    int level = 0);

  /**
   * @brief Collects edges for DOT graph visualization.
   *
   * @param[in] node The node to process.
   * @param[in,out] edges Set to collect edge definitions.
   */
  void get_flow_dotgraph(ActionNode::Ptr node, std::set<std::string> & edges);

  /**
   * @brief Generates DOT graph node definition for a single action node.
   *
   * @param[in] node The node to generate a definition for.
   * @param[in] action_map Map of action IDs to execution information.
   * @param[in] level Indentation level.
   * @return std::string containing the DOT node definition.
   */
  std::string get_node_dotgraph(
    ActionNode::Ptr node, std::shared_ptr<std::map<std::string,
    ActionExecutionInfo>> action_map, int level = 0);

  /**
   * @brief Gets the execution status of an action.
   *
   * @param[in] action The action to check.
   * @param[in] action_map Map of action IDs to execution information.
   * @return Status of the action.
   */
  ActionExecutor::Status get_action_status(
    ActionStamped action,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  /**
   * @brief Adds a legend to the DOT graph.
   *
   * @param[in,out] ss Stream to write the legend to.
   * @param[in] tab_level Indentation level.
   * @param[in] level_counter Counter for cluster IDs.
   * @param[in] node_counter Counter for node IDs.
   */
  void addDotGraphLegend(
    std::stringstream & ss, int tab_level, int level_counter,
    int node_counter);

  /**
   * @brief Helper function to generate indentation.
   *
   * @param[in] level Indentation level.
   * @return std::string with the specified number of tabs.
   */
  std::string t(int level);

  /**
   * @brief Generates the behavior tree XML for executing a single action.
   *
   * @param[in] node The action node.
   * @param[in] l Indentation level.
   * @return std::string containing the behavior tree XML.
   */
  std::string execution_block(const ActionNode::Ptr & node, int l);

  /**
   * @brief Prints a node and its descendants to stderr for debugging.
   *
   * @param[in] node The node to print.
   * @param[in] level Indentation level.
   * @param[in,out] used_nodes Set of nodes already processed.
   */
  void print_node(
    const ActionNode::Ptr & node,
    int level,
    std::set<ActionNode::Ptr> & used_nodes) const;

  /**
   * @brief Prints the entire action graph to stderr for debugging.
   *
   * @param[in] graph The graph to print.
   */
  void print_graph(const plansys2::ActionGraph::Ptr & graph) const;

  /**
   * @brief Prints a node in CSV format to stderr for debugging.
   *
   * @param[in] node The node to print.
   * @param[in] root_num The root number this node belongs to.
   */
  void print_node_csv(const ActionNode::Ptr & node, uint32_t root_num) const;

  /**
   * @brief Prints the entire action graph in CSV format to stderr for debugging.
   *
   * @param[in] graph The graph to print.
   */
  void print_graph_csv(const plansys2::ActionGraph::Ptr & graph) const;

  /**
   * @brief Collects node information in a tabular format.
   *
   * @param[in] node The node to process.
   * @param[in] root_num The root number this node belongs to.
   * @param[out] graph Vector to collect node information.
   */
  void get_node_tabular(
    const plansys2::ActionNode::Ptr & node,
    uint32_t root_num,
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> & graph) const;

  /**
   * @brief Gets the entire action graph in a tabular format.
   *
   * @param[in] graph The graph to process.
   * @return Vector of tuples containing node information.
   */
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> get_graph_tabular(
    const plansys2::ActionGraph::Ptr & graph) const;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  ActionGraph::Ptr graph_;
  std::string bt_;
  std::string bt_action_;
};

}  // namespace plansys2


#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
