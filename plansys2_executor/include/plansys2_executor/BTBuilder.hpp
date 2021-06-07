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

#ifndef PLANSYS2_EXECUTOR__BTBUILDER_HPP_
#define PLANSYS2_EXECUTOR__BTBUILDER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <utility>

#include "std_msgs/msg/empty.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

struct ActionStamped
{
  float time;
  float duration;
  std::shared_ptr<plansys2_msgs::msg::DurativeAction> action;
};

struct GraphNode
{
  using Ptr = std::shared_ptr<GraphNode>;
  static Ptr make_shared() {return std::make_shared<GraphNode>();}

  ActionStamped action;
  int node_num;
  int level_num;

  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;

  std::set<GraphNode::Ptr> in_arcs;
  std::set<GraphNode::Ptr> out_arcs;
};

struct Graph
{
  using Ptr = std::shared_ptr<Graph>;
  static Ptr make_shared() {return std::make_shared<Graph>();}

  std::list<GraphNode::Ptr> roots;
  std::map<float, std::list<GraphNode::Ptr>> levels;
};

class BTBuilder
{
public:
  explicit BTBuilder(rclcpp::Node::SharedPtr node, const std::string & bt_action = "");

  Graph::Ptr get_graph(const plansys2_msgs::msg::Plan & current_plan);
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);
  std::string get_dotgraph(
    Graph::Ptr action_graph, std::shared_ptr<std::map<std::string,
    ActionExecutionInfo>> action_map, bool enable_legend = false,
    bool enable_print_graph = false);

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  std::string bt_action_;

  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan);
  void prune_backwards(GraphNode::Ptr new_node, GraphNode::Ptr node_satisfy);
  void prune_forward(GraphNode::Ptr current, std::list<GraphNode::Ptr> & used_nodes);

  bool is_action_executable(
    const ActionStamped & action,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;
  std::pair<std::string, uint8_t> get_base(
    const plansys2_msgs::msg::Tree & tree,
    uint32_t node_id = 0);
  std::list<GraphNode::Ptr> get_roots(
    std::vector<plansys2::ActionStamped> & action_sequence,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions,
    int & node_counter);
  GraphNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    uint32_t node_id,
    const std::list<GraphNode::Ptr> & roots,
    const GraphNode::Ptr & current);
  GraphNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    uint32_t node_id,
    const GraphNode::Ptr & node,
    const GraphNode::Ptr & current);
  void remove_existing_requirements(
    const plansys2_msgs::msg::Tree & tree,
    std::vector<uint32_t> & requirements,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;
  bool is_parallelizable(
    const plansys2::ActionStamped & action,
    const std::list<GraphNode::Ptr> & ret) const;

  std::string get_flow_tree(
    GraphNode::Ptr node,
    std::list<std::string> & used_nodes,
    int level = 0);
  std::string get_flow_dotgraph(GraphNode::Ptr node, int level = 0);
  std::string get_node_dotgraph(
    GraphNode::Ptr node, std::shared_ptr<std::map<std::string,
    ActionExecutionInfo>> action_map, int level = 0);
  ActionExecutor::Status get_action_status(
    std::shared_ptr<plansys2_msgs::msg::DurativeAction> action,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);
  void addDotGraphLegend(
    std::stringstream & ss, int tab_level, int level_counter,
    int node_counter);

  std::string t(int level);

  std::string execution_block(const GraphNode::Ptr & node, int l);
  void print_node(
    const GraphNode::Ptr & node,
    int level,
    std::set<GraphNode::Ptr> & used_nodes) const;

  void print_graph(const plansys2::Graph::Ptr & graph) const;

  // bool is_predecessor(const PlanItem & op1, const PlanItem & op2);
  // void add_child(GraphNode & parent, GraphNode & new_child);

  // std::shared_ptr<GraphNode> root_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BTBUILDER_HPP_
