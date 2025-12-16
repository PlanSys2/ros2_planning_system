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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_

#include <eigen3/Eigen/Dense>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2
{

/**
 * @brief Structure that represents a state vector in the planning system.
 *
 * Contains the predicates and functions that define a state of the world
 * at a specific point in time during plan execution.
 */
struct StateVec
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
};

/**
 * @class plansys2::STNBTBuilder
 * @brief Behavior tree builder that uses Simple Temporal Networks (STN) to manage temporal constraints.
 *
 * This class implements the BTBuilder interface by constructing an STN from a plan,
 * propagating temporal constraints, and then converting the resulting network into
 * a behavior tree for execution. It handles durative actions with temporal constraints
 * and supports concurrent execution of actions.
 */
class STNBTBuilder : public BTBuilder
{
public:
  /**
   * @brief Constructor for STNBTBuilder.
   */
  STNBTBuilder();

  /**
   * @brief Initializes the builder with behavior tree templates.
   *
   * @param[in] bt_action_1 XML template for action start phase, default empty.
   * @param[in] bt_action_2 XML template for action end phase, default empty.
   * @param[in] precision Precision for time calculations, default 3.
   */
  void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3);

  /**
   * @brief Generates a behavior tree XML from a plan.
   *
   * Builds an STN from the plan, propagates temporal constraints, and converts
   * the resulting network into a behavior tree XML representation.
   *
   * @param[in] current_plan The plan to transform into a behavior tree.
   * @return std::string containing the behavior tree XML, empty if plan is invalid.
   */
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);

  /**
   * @brief Gets the internal graph representation.
   *
   * @return Shared pointer to the STN graph.
   */
  Graph::Ptr get_graph() {return stn_;}

  /**
   * @brief Propagates temporal constraints through the STN.
   *
   * Uses the Floyd-Warshall algorithm to compute all-pairs shortest paths
   * and updates the temporal constraints in the graph.
   *
   * @param[in] stn The graph to propagate constraints through.
   * @return True if the STN is consistent (no negative cycles), false otherwise.
   */
  bool propagate(const Graph::Ptr stn);

  /**
   * @brief Generates a DOT graph representation of the STN for visualization.
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
   * @brief Builds an STN from a plan.
   *
   * Creates a temporal network representing the plan actions and their constraints.
   *
   * @param[in] plan The plan to analyze.
   * @return Shared pointer to the constructed STN.
   */
  Graph::Ptr build_stn(const plansys2_msgs::msg::Plan & plan) const;

  /**
   * @brief Builds a behavior tree from an STN.
   *
   * Traverses the STN and generates the corresponding behavior tree XML.
   *
   * @param[in] stn The STN to convert to a behavior tree.
   * @return std::string containing the behavior tree XML.
   */
  std::string build_bt(const Graph::Ptr stn) const;

  /**
   * @brief Initializes an empty graph with nodes for the initial state, actions, and goal.
   *
   * @param[in] plan The plan to extract actions from.
   * @return Shared pointer to the initialized graph.
   */
  Graph::Ptr init_graph(const plansys2_msgs::msg::Plan & plan) const;

  /**
   * @brief Extracts action information from a plan.
   *
   * @param[in] plan The plan to extract actions from.
   * @return std::vector<ActionStamped> Vector of ActionStamped objects representing the plan actions.
   */
  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan) const;

  /**
   * @brief Gets the set of happening times in a plan.
   *
   * Happening times are timepoints where actions start or end.
   *
   * @param[in] plan The plan to analyze.
   * @return Set of integer timepoints representing happenings.
   */
  std::set<int> get_happenings(const plansys2_msgs::msg::Plan & plan) const;

  /**
   * @brief Finds the happening time for a given time point.
   *
   * @param[in] time The time to find a happening for.
   * @param[in] happenings The set of all happening times.
   * @return Iterator to the happening time, or happenings.end() if not found.
   */
  std::set<int>::iterator get_happening(int time, const std::set<int> & happenings) const;

  /**
   * @brief Finds the previous happening time before a given time point.
   *
   * @param[in] time The time to find the previous happening for.
   * @param[in] happenings The set of all happening times.
   * @return Iterator to the previous happening time, or happenings.end() if not found.
   */
  std::set<int>::iterator get_previous(int time, const std::set<int> & happenings) const;

  /**
   * @brief Creates a simplified representation of the plan with snap actions.
   *
   * Transforms the plan into a multimap where each action is represented by
   * its start, end, and overall points.
   *
   * @param[in] plan The plan to transform.
   * @return Multimap from time points to action snapshots.
   */
  std::multimap<int, ActionStamped> get_simple_plan(const plansys2_msgs::msg::Plan & plan) const;

  /**
   * @brief Computes the state at each happening time in the plan.
   *
   * @param[in] happenings The set of happening times.
   * @param[in] plan The simplified plan representation.
   * @return std::map<int, StateVec> Map from happening times to state vectors.
   */
  std::map<int, StateVec> get_states(
    const std::set<int> & happenings,
    const std::multimap<int, ActionStamped> & plan) const;

  /**
   * @brief Creates a tree representation from a state.
   *
   * @param[in] preds Predicates in the state.
   * @param[in] funcs Functions in the state.
   * @return Tree representing the conjunction of predicates and functions.
   */
  plansys2_msgs::msg::Tree from_state(
    const std::vector<plansys2::Predicate> & preds,
    const std::vector<plansys2::Function> & funcs) const;

  /**
   * @brief Finds the graph nodes corresponding to an action.
   *
   * @param[in] action The action to find nodes for.
   * @param[in] graph The graph to search in.
   * @return std::vector<Node::Ptr> Vector of nodes matching the action.
   */
  std::vector<Node::Ptr> get_nodes(const ActionStamped & action, const Graph::Ptr graph) const;

  /**
   * @brief Checks if a node matches an action.
   *
   * Compares the time and expression of the node's action with the given action.
   *
   * @param[in] node The node to check.
   * @param[in] action The action to match against.
   * @return True if the node matches the action, false otherwise.
   */
  bool is_match(const Node::Ptr node, const ActionStamped & action) const;

  /**
   * @brief Finds the parent actions that an action depends on.
   *
   * Combines actions that satisfy the action's requirements and actions
   * that might threaten the action's execution.
   *
   * @param[in] action The action to find parents for.
   * @param[in] plan The simplified plan representation.
   * @param[in] happenings The set of happening times.
   * @param[in] states The states at each happening time.
   * @return Vector of parent actions.
   */
  std::vector<std::pair<int, ActionStamped>> get_parents(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const std::set<int> & happenings,
    const std::map<int, StateVec> & states) const;

  /**
   * @brief Finds actions that satisfy the requirements of an action.
   *
   * @param[in] action The action to find satisfiers for.
   * @param[in] plan The simplified plan representation.
   * @param[in] happenings The set of happening times.
   * @param[in] states The states at each happening time.
   * @return Vector of actions that satisfy requirements.
   */
  std::vector<std::pair<int, ActionStamped>> get_satisfy(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const std::set<int> & happenings,
    const std::map<int, StateVec> & states) const;

  /**
   * @brief Finds actions that threaten the execution of an action.
   *
   * An action threatens another if it could interfere with its requirements
   * or has contradictory effects.
   *
   * @param[in] action The action to find threats for.
   * @param[in] plan The simplified plan representation.
   * @param[in] happenings The set of happening times.
   * @param[in] states The states at each happening time.
   * @return std::vector<std::pair<int, ActionStamped>> Vector of threatening actions.
   */
  std::vector<std::pair<int, ActionStamped>> get_threat(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const std::set<int> & happenings,
    const std::map<int, StateVec> & states) const;

  /**
   * @brief Checks if an action can be applied in a given state.
   *
   * Determines if there's a valid ordering of actions at the same time point
   * that would allow the action to be executed.
   *
   * @param[in] action The action to check.
   * @param[in] plan The simplified plan representation.
   * @param[in] time The time point to check at.
   * @param[in,out] state The state to check in, updated if a valid ordering is found.
   * @return True if the action can be applied, false otherwise.
   */
  bool can_apply(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const int & time,
    StateVec & state) const;

  /**
   * @brief Computes the difference between two states.
   *
   * Finds predicates and functions that differ between the states.
   *
   * @param[in] X_1 The first state.
   * @param[in] X_2 The second state.
   * @return StateVec containing the differences.
   */
  StateVec get_diff(const StateVec & X_1, const StateVec & X_2) const;

  /**
   * @brief Computes the intersection of two states.
   *
   * Finds predicates and functions that are common to both states.
   *
   * @param[in] X_1 The first state.
   * @param[in] X_2 The second state.
   * @return StateVec containing the intersection.
   */
  StateVec get_intersection(const StateVec & X_1, const StateVec & X_2) const;

  /**
   * @brief Gets the conditions required by an action.
   *
   * Returns different conditions based on the action type (start, end, overall).
   *
   * @param[in] action The action to get conditions for.
   * @return Tree representing the conditions.
   */
  plansys2_msgs::msg::Tree get_conditions(const ActionStamped & action) const;

  /**
   * @brief Gets the effects produced by an action.
   *
   * Returns different effects based on the action type (start, end).
   *
   * @param[in] action The action to get effects for.
   * @return Tree representing the effects.
   */
  plansys2_msgs::msg::Tree get_effects(const ActionStamped & action) const;

  /**
   * @brief Prunes redundant paths in the graph.
   *
   * Removes unnecessary edges that are implied by other paths.
   *
   * @param[in] current The node to prune paths to.
   * @param[in] previous The node to prune paths from.
   */
  void prune_paths(Node::Ptr current, Node::Ptr previous) const;

  /**
   * @brief Checks if there's a path between two nodes.
   *
   * @param[in] current The destination node.
   * @param[in] previous The source node.
   * @return True if a path exists, false otherwise.
   */
  bool check_paths(Node::Ptr current, Node::Ptr previous) const;

  /**
   * @brief Computes the distance matrix for an STN.
   *
   * Creates a matrix where each entry (i,j) represents the maximum allowed
   * time difference between nodes i and j.
   *
   * @param[in] stn The STN to compute distances for.
   * @return Matrix of distances between nodes.
   */
  Eigen::MatrixXd get_distance_matrix(const Graph::Ptr stn) const;

  /**
   * @brief Applies the Floyd-Warshall algorithm to a distance matrix.
   *
   * Updates the matrix to contain shortest paths between all pairs of nodes.
   *
   * @param[in,out] dist The distance matrix to update.
   */
  void floyd_warshall(Eigen::MatrixXd & dist) const;

  /**
   * @brief Generates the behavior tree XML for a node and its descendants.
   *
   * Recursively traverses the graph to generate the BT structure.
   *
   * @param[in] node The node to process.
   * @param[in] prev_node The previous node in the traversal.
   * @param[in,out] used Set of nodes already processed.
   * @param[in] level Indentation level.
   * @return String containing the behavior tree XML.
   */
  std::string get_flow(
    const Node::Ptr node,
    const Node::Ptr prev_node,
    std::set<Node::Ptr> & used,
    const int & level) const;

  /**
   * @brief Generates the BT XML for the start execution block of an action.
   *
   * @param[in] node The action node.
   * @param[in] l Indentation level.
   * @return String containing the BT XML.
   */
  std::string start_execution_block(const Node::Ptr node, const int & l) const;

  /**
   * @brief Generates the BT XML for the end execution block of an action.
   *
   * @param[in] node The action node.
   * @param[in] l Indentation level.
   * @return String containing the BT XML.
   */
  std::string end_execution_block(const Node::Ptr node, const int & l) const;

  /**
   * @brief Collects edge definitions for DOT graph visualization.
   *
   * @param[in] node The node to process.
   * @param[in,out] edges Set to collect edge definitions.
   */
  void get_flow_dotgraph(Node::Ptr node, std::set<std::string> & edges);

  /**
   * @brief Generates DOT graph node definition for a single action node.
   *
   * @param[in] node The node to generate a definition for.
   * @param[in] action_map Map of action IDs to execution information.
   * @return String containing the DOT node definition.
   */
  std::string get_node_dotgraph(
    Node::Ptr node,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

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
   * @param[in] level_counter Counter for cluster IDs.
   * @param[in] node_counter Counter for node IDs.
   * @return String containing the legend definition.
   */
  std::string add_dot_graph_legend(int level_counter, int node_counter);

  /**
   * @brief Prints the entire STN to stderr for debugging.
   *
   * @param[in] graph The graph to print.
   */
  void print_graph(const plansys2::Graph::Ptr graph) const;

  /**
   * @brief Prints a node and its descendants to stderr for debugging.
   *
   * @param[in] node The node to print.
   * @param[in] level Indentation level.
   */
  void print_node(const Node::Ptr node, int level) const;

  /**
   * @brief Prints all arcs in the STN to stderr for debugging.
   *
   * @param[in] graph The graph to print arcs for.
   */
  void print_arcs(const plansys2::Graph::Ptr graph) const;

  /**
   * @brief Checks if an edge represents the end of a durative action.
   *
   * @param[in] edge The edge to check.
   * @param[in] action The action to compare against.
   * @return True if the edge connects to the end node of the action.
   */
  bool is_end(
    const std::tuple<Node::Ptr, double, double> & edge,
    const ActionStamped & action) const;

  /**
   * @brief Helper function to generate indentation.
   *
   * @param[in] level Indentation level.
   * @return std::string with the specified number of tabs.
   */
  std::string t(const int & level) const;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  Graph::Ptr stn_;
  std::string bt_start_action_;
  std::string bt_end_action_;
  int action_time_precision_;
};

}  // namespace plansys2


#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_
