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
#include <memory>
#include <vector>
#include <set>
#include <algorithm>
#include <list>
#include <tuple>
#include <map>
#include <utility>

#include "plansys2_executor/BTBuilder.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.h"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

BTBuilder::BTBuilder(
  rclcpp::Node::SharedPtr node,
  const std::string & bt_action)
{
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

  if (bt_action != "") {
    bt_action_ = bt_action;
  } else {
    bt_action_ =
      R""""(<Sequence name="ACTION_ID">
WAIT_AT_START_ACTIONS
  <ApplyAtStartEffect action="ACTION_ID"/>
  <ReactiveSequence name="ACTION_ID">
    <CheckOverAllReq action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
</Sequence>
)"""";
  }
}

bool
BTBuilder::is_action_executable(
  const ActionStamped & action,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  return check(action.action->at_start_requirements, predicates, functions) &&
         check(action.action->over_all_requirements, predicates, functions) &&
         check(action.action->at_end_requirements, predicates, functions);
}

std::pair<std::string, uint8_t>
BTBuilder::get_base(
  const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id)
{
  std::string base_expr;
  uint8_t base_type = plansys2_msgs::msg::Node::UNKNOWN;

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::NOT: {
        auto child_id = tree.nodes[node_id].children[0];
        base_expr = parser::pddl::toString(tree, child_id);
        base_type = plansys2_msgs::msg::Node::PREDICATE;
        break;
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        base_expr = parser::pddl::toString(tree, node_id);
        base_type = plansys2_msgs::msg::Node::PREDICATE;
        break;
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        auto child_id = tree.nodes[node_id].children[0];
        if (tree.nodes[child_id].node_type == plansys2_msgs::msg::Node::FUNCTION) {
          base_expr = parser::pddl::toString(tree, child_id);
          base_type = plansys2_msgs::msg::Node::FUNCTION;
        } else {
          std::cerr << "get_base: Error parsing expresion [" <<
            parser::pddl::toString(tree, node_id) << "]" << std::endl;
        }
        break;
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        auto child_id = tree.nodes[node_id].children[0];
        base_expr = parser::pddl::toString(tree, child_id);
        base_type = plansys2_msgs::msg::Node::FUNCTION;
        break;
      }

    default: {
        std::cerr << "get_base: Error parsing expresion [" <<
          parser::pddl::toString(tree) << "]" << std::endl;
        break;
      }
  }

  return std::make_pair(base_expr, base_type);
}

GraphNode::Ptr
BTBuilder::get_node_satisfy(
  const plansys2_msgs::msg::Tree & requirement,
  uint32_t node_id,
  const GraphNode::Ptr & node,
  const GraphNode::Ptr & current)
{
  if (node == current) {
    return nullptr;
  }

  std::pair<std::string, uint8_t> requirement_base = get_base(requirement, node_id);

  GraphNode::Ptr ret = nullptr;
  std::vector<uint32_t> at_start_effects =
    parser::pddl::getSubtrees(node->action.action->at_start_effects);
  std::vector<uint32_t> at_end_effects =
    parser::pddl::getSubtrees(node->action.action->at_end_effects);

  std::vector<uint32_t> at_start_requirements =
    parser::pddl::getSubtrees(node->action.action->at_start_requirements);
  std::vector<uint32_t> over_all_requirements =
    parser::pddl::getSubtrees(node->action.action->over_all_requirements);
  std::vector<uint32_t> at_end_requirements =
    parser::pddl::getSubtrees(node->action.action->at_end_requirements);

  for (const auto & effect : at_end_effects) {
    std::pair<std::string, uint8_t> base = get_base(node->action.action->at_end_effects, effect);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions, node_id)) {
      ret = node;
    }
  }

  for (const auto & effect : at_start_effects) {
    std::pair<std::string, uint8_t> base = get_base(node->action.action->at_start_effects, effect);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions, node_id)) {
      ret = node;
    }
  }

  for (const auto & req : at_start_requirements) {
    std::pair<std::string,
      uint8_t> base = get_base(node->action.action->at_start_requirements, req);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions, node_id)) {
      ret = node;
    }
  }

  for (const auto & req : over_all_requirements) {
    std::pair<std::string,
      uint8_t> base = get_base(node->action.action->over_all_requirements, req);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions, node_id)) {
      ret = node;
    }
  }

  for (const auto & req : at_end_requirements) {
    std::pair<std::string, uint8_t> base = get_base(node->action.action->at_end_requirements, req);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions, node_id)) {
      ret = node;
    }
  }

  for (const auto & arc : node->out_arcs) {
    auto node_ret = get_node_satisfy(requirement, node_id, arc, current);

    if (node_ret != nullptr) {
      ret = node_ret;
    }
  }

  return ret;
}

bool
BTBuilder::is_parallelizable(
  const plansys2::ActionStamped & action,
  const std::list<GraphNode::Ptr> & ret) const
{
  std::vector<plansys2_msgs::msg::Node> action_at_start_requirements;
  parser::pddl::getPredicates(action_at_start_requirements, action.action->at_start_requirements);

  for (const auto & other : ret) {
    std::vector<plansys2_msgs::msg::Node> other_over_all_requirements;
    parser::pddl::getPredicates(
      other_over_all_requirements,
      other->action.action->over_all_requirements);

    for (const auto & prev_over_all_req : other_over_all_requirements) {
      for (const auto & action_at_start_req : action_at_start_requirements) {
        if (parser::pddl::toString(prev_over_all_req) ==
          parser::pddl::toString(action_at_start_req) &&
          prev_over_all_req.negate == action_at_start_req.negate)
        {
          return false;
        }
      }
    }
  }

  return true;
}

GraphNode::Ptr
BTBuilder::get_node_satisfy(
  const plansys2_msgs::msg::Tree & requirement,
  uint32_t node_id,
  const std::list<GraphNode::Ptr> & roots,
  const GraphNode::Ptr & current)
{
  GraphNode::Ptr ret;
  for (const auto & node : roots) {
    auto node_ret = get_node_satisfy(requirement, node_id, node, current);
    if (node_ret != nullptr) {
      ret = node_ret;
    }
  }

  return ret;
}

std::list<GraphNode::Ptr>
BTBuilder::get_roots(
  std::vector<plansys2::ActionStamped> & action_sequence,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  int & node_counter)
{
  std::list<GraphNode::Ptr> ret;

  auto it = action_sequence.begin();
  while (it != action_sequence.end()) {
    const auto & action = *it;
    if (is_action_executable(action, predicates, functions) && is_parallelizable(action, ret)) {
      auto new_root = GraphNode::make_shared();
      new_root->action = action;
      new_root->node_num = node_counter++;
      new_root->level_num = 0;

      ret.push_back(new_root);
      it = action_sequence.erase(it);
    } else {
      break;
    }
  }

  return ret;
}

void
BTBuilder::remove_existing_requirements(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<uint32_t> & requirements,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  auto it = requirements.begin();
  while (it != requirements.end()) {
    if (check(tree, predicates, functions, *it)) {
      it = requirements.erase(it);
    } else {
      ++it;
    }
  }
}

void
BTBuilder::prune_backwards(GraphNode::Ptr new_node, GraphNode::Ptr node_satisfy)
{
  // Repeat prune to the roots
  for (auto & in : node_satisfy->in_arcs) {
    prune_backwards(new_node, in);
  }

  auto it = node_satisfy->out_arcs.begin();
  while (it != node_satisfy->out_arcs.end()) {
    if (*it == new_node) {
      (*it)->in_arcs.erase(*it);
      it = node_satisfy->out_arcs.erase(it);
    } else {
      ++it;
    }
  }
}

void
BTBuilder::prune_forward(GraphNode::Ptr current, std::list<GraphNode::Ptr> & used_nodes)
{
  auto it = current->out_arcs.begin();
  while (it != current->out_arcs.end()) {
    if (std::find(used_nodes.begin(), used_nodes.end(), *it) != used_nodes.end()) {
      it = current->out_arcs.erase(it);
    } else {
      prune_forward(*it, used_nodes);
      used_nodes.push_back(*it);

      ++it;
    }
  }
}

Graph::Ptr
BTBuilder::get_graph(const plansys2_msgs::msg::Plan & current_plan)
{
  int node_counter = 0;
  int level_counter = 0;
  auto graph = Graph::make_shared();

  auto action_sequence = get_plan_actions(current_plan);
  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();

  graph->roots = get_roots(action_sequence, predicates, functions, node_counter);

  // Apply root actions
  for (auto & action_node : graph->roots) {
    // Create a local copy of the state
    action_node->predicates = problem_client_->getPredicates();
    action_node->functions = problem_client_->getFunctions();

    // Apply the effects to the local node state
    apply(
      action_node->action.action->at_start_effects,
      action_node->predicates, action_node->functions);
    apply(
      action_node->action.action->at_end_effects,
      action_node->predicates, action_node->functions);

    // Apply the effects to the global state
    apply(
      action_node->action.action->at_start_effects,
      predicates, functions);
    apply(
      action_node->action.action->at_end_effects,
      predicates, functions);
  }


  // Build the rest of the graph
  while (!action_sequence.empty()) {
    auto new_node = GraphNode::make_shared();
    new_node->action = *action_sequence.begin();
    new_node->node_num = node_counter++;
    float time = new_node->action.time;

    auto level = graph->levels.find(time);
    if (level == graph->levels.end()) {
      level_counter++;
      std::list<GraphNode::Ptr> new_level;
      new_level.push_back(new_node);
      graph->levels.insert({time, new_level});
    } else {
      level->second.push_back(new_node);
    }
    new_node->level_num = level_counter;

    std::vector<uint32_t> at_start_requirements =
      parser::pddl::getSubtrees(action_sequence.begin()->action->at_start_requirements);
    std::vector<uint32_t> over_all_requirements =
      parser::pddl::getSubtrees(action_sequence.begin()->action->over_all_requirements);
    std::vector<uint32_t> at_end_requirements =
      parser::pddl::getSubtrees(action_sequence.begin()->action->at_end_requirements);

    auto it_at_start = at_start_requirements.begin();
    while (it_at_start != at_start_requirements.end()) {
      auto node_satisfy =
        get_node_satisfy(
        action_sequence.begin()->action->at_start_requirements,
        *it_at_start,
        graph->roots,
        new_node);
      if (node_satisfy != nullptr) {
        prune_backwards(new_node, node_satisfy);

        // Create the connections to the parent node
        new_node->in_arcs.insert(node_satisfy);
        node_satisfy->out_arcs.insert(new_node);

        // Copy the state from the parent node
        new_node->predicates = node_satisfy->predicates;
        new_node->functions = node_satisfy->functions;

        // Apply the effects of the new node
        apply(
          new_node->action.action->at_start_effects,
          new_node->predicates, new_node->functions);
        apply(
          new_node->action.action->at_end_effects,
          new_node->predicates, new_node->functions);

        it_at_start = at_start_requirements.erase(it_at_start);
      } else {
        ++it_at_start;
      }
    }

    auto it_over_all = over_all_requirements.begin();
    while (it_over_all != over_all_requirements.end()) {
      auto node_satisfy =
        get_node_satisfy(
        action_sequence.begin()->action->over_all_requirements,
        *it_over_all,
        graph->roots,
        new_node);
      if (node_satisfy != nullptr) {
        prune_backwards(new_node, node_satisfy);

        // Create the connections to the parent node
        new_node->in_arcs.insert(node_satisfy);
        node_satisfy->out_arcs.insert(new_node);

        // Copy the state from the parent node
        new_node->predicates = node_satisfy->predicates;
        new_node->functions = node_satisfy->functions;

        // Apply the effects of the new node
        apply(
          new_node->action.action->at_start_effects,
          new_node->predicates, new_node->functions);
        apply(
          new_node->action.action->at_end_effects,
          new_node->predicates, new_node->functions);

        it_over_all = over_all_requirements.erase(it_over_all);
      } else {
        ++it_over_all;
      }
    }

    auto it_at_end = at_end_requirements.begin();
    while (it_at_end != at_end_requirements.end()) {
      auto node_satisfy =
        get_node_satisfy(
        action_sequence.begin()->action->at_end_requirements,
        *it_at_end,
        graph->roots,
        new_node);
      if (node_satisfy != nullptr) {
        prune_backwards(new_node, node_satisfy);

        // Create the connections to the parent node
        new_node->in_arcs.insert(node_satisfy);
        node_satisfy->out_arcs.insert(new_node);

        // Copy the state from the parent node
        new_node->predicates = node_satisfy->predicates;
        new_node->functions = node_satisfy->functions;

        // Apply the effects of the new node
        apply(
          new_node->action.action->at_start_effects,
          new_node->predicates, new_node->functions);
        apply(
          new_node->action.action->at_end_effects,
          new_node->predicates, new_node->functions);

        it_at_end = at_end_requirements.erase(it_at_end);
      } else {
        ++it_at_end;
      }
    }

    remove_existing_requirements(
      action_sequence.begin()->action->at_start_requirements, at_start_requirements, predicates,
      functions);
    remove_existing_requirements(
      action_sequence.begin()->action->over_all_requirements, over_all_requirements, predicates,
      functions);
    remove_existing_requirements(
      action_sequence.begin()->action->at_end_requirements, at_end_requirements, predicates,
      functions);

    for (const auto & req : at_start_requirements) {
      std::cerr << "===> [" << parser::pddl::toString(
        action_sequence.begin()->action->at_start_requirements, req) << "]" << std::endl;
    }

    assert(at_start_requirements.empty());
    assert(over_all_requirements.empty());
    assert(at_end_requirements.empty());

    action_sequence.erase(action_sequence.begin());
  }

  std::list<GraphNode::Ptr> used_nodes;
  for (auto & root : graph->roots) {
    prune_forward(root, used_nodes);
  }
  return graph;
}

std::string
BTBuilder::get_tree(const plansys2_msgs::msg::Plan & current_plan)
{
  auto action_graph = get_graph(current_plan);

  std::string bt_plan;

  std::list<std::string> used_nodes;

  if (action_graph->roots.size() > 1) {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n" +
      t(2) + "<Parallel success_threshold=\"" + std::to_string(action_graph->roots.size()) +
      "\" failure_threshold=\"1\">\n";

    for (const auto & node : action_graph->roots) {
      bt_plan = bt_plan + get_flow_tree(node, used_nodes, 3);
    }

    bt_plan = bt_plan + t(2) + "</Parallel>\n" +
      t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n";

    bt_plan = bt_plan + get_flow_tree(*action_graph->roots.begin(), used_nodes, 2);

    bt_plan = bt_plan + t(1) + "</BehaviorTree>\n</root>\n";
  }

  return bt_plan;
}

std::string
BTBuilder::get_dotgraph(
  Graph::Ptr action_graph, std::shared_ptr<std::map<std::string,
  ActionExecutionInfo>> action_map, bool enable_legend,
  bool enable_print_graph)
{
  if (enable_print_graph) {
    print_graph(action_graph);
  }

  // create xdot graph
  std::stringstream ss;
  ss << "digraph plan {\n";

  int tab_level = 1;
  // dotgraph formatting options
  ss << t(tab_level);
  ss << "node[shape=box];\n";
  ss << t(tab_level);
  ss << "rankdir=TB;\n";

  // define all the levels and nodes
  ss << t(tab_level);
  ss << "subgraph cluster_0 {\n";

  tab_level = 2;
  ss << t(tab_level);
  ss << "label = \"Time: 0.0\";\n";
  ss << t(tab_level);
  ss << "style = rounded;\n";
  ss << t(tab_level);
  ss << "color = yellow3;\n";
  ss << t(tab_level);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(tab_level);
  ss << "labeljust = l;\n";

  tab_level = 3;
  for (auto & node : action_graph->roots) {
    ss << get_node_dotgraph(node, action_map, tab_level);
  }
  tab_level = 2;

  ss << t(tab_level);
  ss << "}\n";

  int max_level = 0;
  int max_node = 0;
  for (auto & level : action_graph->levels) {
    if (!level.second.empty()) {
      ss << t(tab_level);
      ss << "subgraph cluster_" << level.second.front()->level_num << " {\n";
      max_level = std::max(max_level, level.second.front()->level_num);

      tab_level = 2;
      ss << t(tab_level);
      ss << "label = \"Time: " << level.second.front()->action.time << "\";\n";
      ss << t(tab_level);
      ss << "style = rounded;\n";
      ss << t(tab_level);
      ss << "color = yellow3;\n";
      ss << t(tab_level);
      ss << "bgcolor = lemonchiffon;\n";
      ss << t(tab_level);
      ss << "labeljust = l;\n";

      tab_level = 3;
      for (auto & node : level.second) {
        max_node = std::max(max_node, node->node_num);
        ss << get_node_dotgraph(node, action_map, tab_level);
      }
      tab_level = 2;

      ss << t(tab_level);
      ss << "}\n";
    }
  }

  tab_level = 1;
  // define the edges
  for (const auto & graph_root : action_graph->roots) {
    ss << get_flow_dotgraph(graph_root, tab_level);
  }

  if (enable_legend) {
    max_level++;
    max_node++;
    addDotGraphLegend(ss, tab_level, max_level, max_node);
  }

  ss << "}";

  return ss.str();
}

std::string
BTBuilder::get_flow_tree(
  GraphNode::Ptr node,
  std::list<std::string> & used_nodes,
  int level)
{
  std::string ret;
  int l = level;

  const std::string action_id = "(" + parser::pddl::nameActionsToString(node->action.action) +
    "):" +
    std::to_string(static_cast<int>(node->action.time * 1000));

  if (std::find(used_nodes.begin(), used_nodes.end(), action_id) != used_nodes.end()) {
    return t(l) + "<WaitAction action=\"" + action_id + "\"/>\n";
  }

  used_nodes.push_back(action_id);

  if (node->out_arcs.size() == 0) {
    ret = ret + execution_block(node, l);
  } else if (node->out_arcs.size() == 1) {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    for (const auto & child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, used_nodes, l + 1);
    }

    ret = ret + t(l) + "</Sequence>\n";
  } else {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    ret = ret + t(l + 1) +
      "<Parallel success_threshold=\"" + std::to_string(node->out_arcs.size()) +
      "\" failure_threshold=\"1\">\n";

    for (const auto & child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, used_nodes, l + 2);
    }

    ret = ret + t(l + 1) + "</Parallel>\n";
    ret = ret + t(l) + "</Sequence>\n";
  }

  return ret;
}

std::string
BTBuilder::get_flow_dotgraph(
  GraphNode::Ptr node,
  int level)
{
  std::stringstream ss;

  for (const auto & child_node : node->out_arcs) {
    ss << t(level);
    ss << node->node_num << "->" << child_node->node_num << ";\n";
    ss << get_flow_dotgraph(child_node, level);
  }

  return ss.str();
}

std::string
BTBuilder::get_node_dotgraph(
  GraphNode::Ptr node, std::shared_ptr<std::map<std::string,
  ActionExecutionInfo>> action_map, int level)
{
  std::stringstream ss;
  ss << t(level);
  ss << node->node_num << " [label=\"" << parser::pddl::nameActionsToString(node->action.action) <<
    "\"";
  ss << "labeljust=c,style=filled";

  auto status = get_action_status(node->action.action, action_map);
  switch (status) {
    case ActionExecutor::RUNNING:
      ss << ",color=blue,fillcolor=skyblue";
      break;
    case ActionExecutor::SUCCESS:
      ss << ",color=green4,fillcolor=seagreen2";
      break;
    case ActionExecutor::FAILURE:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::CANCELLED:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::IDLE:
    case ActionExecutor::DEALING:
    default:
      ss << ",color=yellow3,fillcolor=lightgoldenrod1";
      break;
  }
  ss << "];\n";
  return ss.str();
}

ActionExecutor::Status BTBuilder::get_action_status(
  std::shared_ptr<plansys2_msgs::msg::DurativeAction> action,
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map)
{
  for (const auto & action_pair : *action_map) {
    if (parser::pddl::nameActionsToString(action_pair.second.durative_action_info) ==
      parser::pddl::nameActionsToString(action))
    {
      return action_pair.second.action_executor->get_internal_status();
    }
  }
  return ActionExecutor::IDLE;
}

void BTBuilder::addDotGraphLegend(
  std::stringstream & ss, int tab_level, int level_counter,
  int node_counter)
{
  int legend_counter = level_counter;
  int legend_node_counter = node_counter;
  ss << t(tab_level);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  tab_level++;
  ss << t(tab_level);
  ss << "label = \"Legend\";\n";

  ss << t(tab_level);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  tab_level++;
  ss << t(tab_level);
  ss << "label = \"Plan Timestep (sec): X.X\";\n";
  ss << t(tab_level);
  ss << "style = rounded;\n";
  ss << t(tab_level);
  ss << "color = yellow3;\n";
  ss << t(tab_level);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(tab_level);
  ss << "labeljust = l;\n";
  ss << t(tab_level);
  ss << legend_node_counter++ <<
    " [label=\n\"Finished action\n\",labeljust=c,style=filled,color=green4,fillcolor=seagreen2];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ <<
    " [label=\n\"Failed action\n\",labeljust=c,style=filled,color=red,fillcolor=pink];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ <<
    " [label=\n\"Current action\n\",labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ << " [label=\n\"Future action\n\",labeljust=c,style=filled," <<
    "color=yellow3,fillcolor=lightgoldenrod1];\n";
  tab_level--;
  ss << t(tab_level);
  ss << "}\n";

  ss << t(tab_level);
  for (int i = node_counter; i < legend_node_counter; i++) {
    if (i > node_counter) {
      ss << "->";
    }
    ss << i;
  }
  ss << " [style=invis];\n";

  tab_level--;
  ss << t(tab_level);
  ss << "}\n";
}

std::string
BTBuilder::t(int level)
{
  std::string ret;
  for (int i = 0; i < level; i++) {
    ret = ret + "  ";
  }
  return ret;
}

void replace(std::string & str, const std::string & from, const std::string & to)
{
  size_t start_pos = std::string::npos;
  while ((start_pos = str.find(from)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
  }
}

std::string
BTBuilder::execution_block(const GraphNode::Ptr & node, int l)
{
  const auto & action = node->action;
  std::string ret;
  std::string ret_aux = bt_action_;
  const std::string action_id = "(" + parser::pddl::nameActionsToString(action.action) + "):" +
    std::to_string(static_cast<int>(action.time * 1000));

  std::string wait_actions;
  for (const auto & previous_node : node->in_arcs) {
    const std::string parent_action_id = "(" +
      parser::pddl::nameActionsToString(previous_node->action.action) + "):" +
      std::to_string(static_cast<int>( previous_node->action.time * 1000));
    wait_actions = wait_actions + t(1) + "<WaitAtStartReq action=\"" + parent_action_id + "\"/>";

    if (previous_node != *node->in_arcs.rbegin()) {
      wait_actions = wait_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);
  replace(ret_aux, "WAIT_AT_START_ACTIONS", wait_actions);

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;
}

std::vector<ActionStamped>
BTBuilder::get_plan_actions(const plansys2_msgs::msg::Plan & plan)
{
  std::vector<ActionStamped> ret;

  for (auto & item : plan.items) {
    ActionStamped action_stamped;

    action_stamped.time = item.time;
    action_stamped.duration = item.duration;
    action_stamped.action =
      domain_client_->getDurativeAction(
      get_action_name(item.action), get_action_params(item.action));

    ret.push_back(action_stamped);
  }

  return ret;
}

void
BTBuilder::print_node(
  const plansys2::GraphNode::Ptr & node,
  int level,
  std::set<plansys2::GraphNode::Ptr> & used_nodes) const
{
  std::cerr << std::string(level, '\t') << "[" << node->action.time << "] ";
  std::cerr << node->action.action->name << " ";
  for (const auto & param : node->action.action->parameters) {
    std::cerr << param.name << " ";
  }
  std::cerr << " in arcs " << node->in_arcs.size() << "  ";
  std::cerr << " out arcs " << node->out_arcs.size() << std::endl;

  for (const auto & out : node->out_arcs) {
    print_node(out, level + 1, used_nodes);
  }
}

void
BTBuilder::print_graph(const plansys2::Graph::Ptr & graph) const
{
  std::set<plansys2::GraphNode::Ptr> used_nodes;
  for (const auto & root : graph->roots) {
    print_node(root, 0, used_nodes);
  }
}

}  // namespace plansys2
