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

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

BTBuilder::BTBuilder(
  rclcpp::Node::SharedPtr node)
{
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(node);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(node);
}

void
BTBuilder::init_predicates(
  std::set<std::string> & predicates,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  for (const auto & predicate : problem_client->getPredicates()) {
    predicates.insert(predicate.toString());
  }
}

void
BTBuilder::init_functions(
  std::map<std::string, double> & functions,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  for (const auto & function : problem_client->getFunctions()) {
    std::optional<parser::pddl::tree::Function> func =
      problem_client->getFunction(function.toString());
    if (func.has_value()) {
      functions.insert(std::make_pair(func.value().toString(), func.value().value));
    } else {
      std::cerr << "init_functions: Error retrieving function [" <<
        function.toString() << "]" << std::endl;
    }
  }
}

bool
BTBuilder::is_action_executable(
  const ActionStamped & action,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions) const
{
  return check(action.action->at_start_requirements.root_, predicates, functions) &&
         check(action.action->over_all_requirements.root_, predicates, functions) &&
         check(action.action->at_end_requirements.root_, predicates, functions);
}

std::pair<std::string, parser::pddl::tree::NodeType>
BTBuilder::get_base(
  const std::shared_ptr<parser::pddl::tree::TreeNode> tree_node)
{
  std::string base_expr;
  parser::pddl::tree::NodeType base_type = parser::pddl::tree::UNKNOWN_NODE_TYPE;

  switch (tree_node->type_) {
    case parser::pddl::tree::NOT: {
        std::shared_ptr<parser::pddl::tree::NotNode> pn_not =
          std::dynamic_pointer_cast<parser::pddl::tree::NotNode>(tree_node);
        base_expr = pn_not->op->toString();
        base_type = parser::pddl::tree::PREDICATE;
        break;
      }

    case parser::pddl::tree::PREDICATE: {
        base_expr = tree_node->toString();
        base_type = parser::pddl::tree::PREDICATE;
        break;
      }

    case parser::pddl::tree::EXPRESSION: {
        std::shared_ptr<parser::pddl::tree::ExpressionNode> expr =
          std::dynamic_pointer_cast<parser::pddl::tree::ExpressionNode>(tree_node);
        if (expr->ops[0]->type_ == parser::pddl::tree::FUNCTION) {
          std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
            std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(expr->ops[0]);
          base_expr = func_node->toString();
          base_type = parser::pddl::tree::FUNCTION;
        } else {
          std::cerr << "get_base: Error parsing expresion [" <<
            tree_node->toString() << "]" << std::endl;
        }
        break;
      }

    case parser::pddl::tree::FUNCTION_MODIFIER: {
        std::shared_ptr<parser::pddl::tree::FunctionModifierNode> func_mod =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionModifierNode>(tree_node);
        std::shared_ptr<parser::pddl::tree::FunctionNode> func_node =
          std::dynamic_pointer_cast<parser::pddl::tree::FunctionNode>(func_mod->ops[0]);
        base_expr = func_node->toString();
        base_type = parser::pddl::tree::FUNCTION;
        break;
      }

    default: {
        std::cerr << "get_base: Error parsing expresion [" <<
          tree_node->toString() << "]" << std::endl;
        break;
      }
  }

  return std::make_pair(base_expr, base_type);
}

GraphNode::Ptr
BTBuilder::get_node_satisfy(
  const std::shared_ptr<parser::pddl::tree::TreeNode> requirement,
  const GraphNode::Ptr & node,
  const GraphNode::Ptr & current)
{
  if (node == current) {
    return nullptr;
  }

  std::pair<std::string, parser::pddl::tree::NodeType> requirement_base =
    get_base(requirement);

  GraphNode::Ptr ret = nullptr;
  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> at_start_effects =
    get_subtrees(node->action.action->at_start_effects.root_);
  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> at_end_effects =
    get_subtrees(node->action.action->at_end_effects.root_);

  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> at_start_requirements =
    get_subtrees(node->action.action->at_start_requirements.root_);
  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> over_all_requirements =
    get_subtrees(node->action.action->over_all_requirements.root_);
  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> at_end_requirements =
    get_subtrees(node->action.action->at_end_requirements.root_);

  for (const auto & effect : at_end_effects) {
    std::pair<std::string, parser::pddl::tree::NodeType> base = get_base(effect);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions)) {
      ret = node;
    }
  }

  for (const auto & effect : at_start_effects) {
    std::pair<std::string, parser::pddl::tree::NodeType> base = get_base(effect);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions)) {
      ret = node;
    }
  }

  for (const auto & req : at_start_requirements) {
    std::pair<std::string, parser::pddl::tree::NodeType> base = get_base(req);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions)) {
      ret = node;
    }
  }

  for (const auto & req : over_all_requirements) {
    std::pair<std::string, parser::pddl::tree::NodeType> base = get_base(req);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions)) {
      ret = node;
    }
  }

  for (const auto & req : at_end_requirements) {
    std::pair<std::string, parser::pddl::tree::NodeType> base = get_base(req);
    if (base != requirement_base) {
      continue;
    }

    if (check(requirement, node->predicates, node->functions)) {
      ret = node;
    }
  }

  for (const auto & arc : node->out_arcs) {
    auto node_ret = get_node_satisfy(requirement, arc, current);

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
  std::vector<parser::pddl::tree::Predicate> action_at_start_requirements;
  action.action->at_start_requirements.getPredicates(action_at_start_requirements);

  for (const auto & other : ret) {
    std::vector<parser::pddl::tree::Predicate> other_over_all_requirements;
    other->action.action->over_all_requirements.getPredicates(other_over_all_requirements);

    for (const auto & prev_over_all_req : other_over_all_requirements) {
      for (const auto & action_at_start_req : action_at_start_requirements) {
        if (prev_over_all_req.toString() == action_at_start_req.toString() &&
          prev_over_all_req.negative == action_at_start_req.negative)
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
  const std::shared_ptr<parser::pddl::tree::TreeNode> requirement,
  const std::list<GraphNode::Ptr> & roots,
  const GraphNode::Ptr & current)
{
  GraphNode::Ptr ret;
  for (const auto & node : roots) {
    auto node_ret = get_node_satisfy(requirement, node, current);
    if (node_ret != nullptr) {
      ret = node_ret;
    }
  }

  return ret;
}

std::list<GraphNode::Ptr>
BTBuilder::get_roots(
  std::vector<plansys2::ActionStamped> & action_sequence,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions,
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
  std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> & requirements,
  std::set<std::string> & predicates,
  std::map<std::string, double> & functions) const
{
  auto it = requirements.begin();
  while (it != requirements.end()) {
    if (check(*it, predicates, functions)) {
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
BTBuilder::get_graph(const Plan & current_plan)
{
  std::set<std::string> predicates;
  std::map<std::string, double> functions;
  int node_counter = 0;
  int level_counter = 0;
  auto graph = Graph::make_shared();

  auto action_sequence = get_plan_actions(current_plan);
  init_predicates(predicates, problem_client_);
  init_functions(functions, problem_client_);

  graph->roots = get_roots(action_sequence, predicates, functions, node_counter);

  // Apply root actions
  for (auto & action_node : graph->roots) {
    // Create a local copy of the state
    init_predicates(action_node->predicates, problem_client_);
    init_functions(action_node->functions, problem_client_);

    // Apply the effects to the local node state
    apply(
      action_node->action.action->at_start_effects.root_,
      action_node->predicates, action_node->functions);
    apply(
      action_node->action.action->at_end_effects.root_,
      action_node->predicates, action_node->functions);

    // Apply the effects to the global state
    apply(
      action_node->action.action->at_start_effects.root_,
      predicates, functions);
    apply(
      action_node->action.action->at_end_effects.root_,
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

    std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> at_start_requirements =
      get_subtrees(action_sequence.begin()->action->at_start_requirements.root_);
    std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> over_all_requirements =
      get_subtrees(action_sequence.begin()->action->over_all_requirements.root_);
    std::vector<std::shared_ptr<parser::pddl::tree::TreeNode>> at_end_requirements =
      get_subtrees(action_sequence.begin()->action->at_end_requirements.root_);

    auto it_at_start = at_start_requirements.begin();
    while (it_at_start != at_start_requirements.end()) {
      auto node_satisfy = get_node_satisfy(*it_at_start, graph->roots, new_node);
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
          new_node->action.action->at_start_effects.root_,
          new_node->predicates, new_node->functions);
        apply(
          new_node->action.action->at_end_effects.root_,
          new_node->predicates, new_node->functions);

        it_at_start = at_start_requirements.erase(it_at_start);
      } else {
        ++it_at_start;
      }
    }

    auto it_over_all = over_all_requirements.begin();
    while (it_over_all != over_all_requirements.end()) {
      auto node_satisfy = get_node_satisfy(*it_over_all, graph->roots, new_node);
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
          new_node->action.action->at_start_effects.root_,
          new_node->predicates, new_node->functions);
        apply(
          new_node->action.action->at_end_effects.root_,
          new_node->predicates, new_node->functions);

        it_over_all = over_all_requirements.erase(it_over_all);
      } else {
        ++it_over_all;
      }
    }

    auto it_at_end = at_end_requirements.begin();
    while (it_at_end != at_end_requirements.end()) {
      auto node_satisfy = get_node_satisfy(*it_at_end, graph->roots, new_node);
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
          new_node->action.action->at_start_effects.root_,
          new_node->predicates, new_node->functions);
        apply(
          new_node->action.action->at_end_effects.root_,
          new_node->predicates, new_node->functions);

        it_at_end = at_end_requirements.erase(it_at_end);
      } else {
        ++it_at_end;
      }
    }

    remove_existing_requirements(at_start_requirements, predicates, functions);
    remove_existing_requirements(over_all_requirements, predicates, functions);
    remove_existing_requirements(at_end_requirements, predicates, functions);

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
BTBuilder::get_tree(const Plan & current_plan)
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
BTBuilder::get_dotgraph(const Plan & current_plan)
{
  auto action_graph = get_graph(current_plan);

  print_graph(action_graph);

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
    ss << t(tab_level);
    ss << node->node_num << " [label=\"" << node->action.action->name_actions_to_string() << "\"";
    ss << "labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
  }
  tab_level = 2;

  ss << t(tab_level);
  ss << "}\n";

  for (auto & level : action_graph->levels) {
    if (!level.second.empty()) {
      ss << t(tab_level);
      ss << "subgraph cluster_" << level.second.front()->level_num << " {\n";

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
        ss << t(tab_level);
        ss << node->node_num << " [label=\"" << node->action.action->name_actions_to_string() <<
          "\"";
        ss << "labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
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

  const std::string action_id = "(" + node->action.action->name_actions_to_string() + "):" +
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
BTBuilder::t(int level)
{
  std::string ret;
  for (int i = 0; i < level; i++) {
    ret = ret + "  ";
  }
  return ret;
}

std::string
BTBuilder::execution_block(const GraphNode::Ptr & node, int l)
{
  const auto & action = node->action;
  std::string ret;
  const std::string action_id = "(" + action.action->name_actions_to_string() + "):" +
    std::to_string(static_cast<int>(action.time * 1000));

  ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";

  for (const auto & previous_node : node->in_arcs) {
    const std::string parent_action_id = "(" +
      previous_node->action.action->name_actions_to_string() + "):" +
      std::to_string(static_cast<int>( previous_node->action.time * 1000));
    ret = ret + t(l + 1) + "<WaitAtStartReq action=\"" + parent_action_id + "\"/>\n";
  }

  ret = ret + t(l + 1) + "<ApplyAtStartEffect action=\"" + action_id + "\"/>\n";
  ret = ret + t(l + 1) + "<ReactiveSequence name=\"" + action_id + "\">\n";
  ret = ret + t(l + 2) + "<CheckOverAllReq action=\"" + action_id + "\"/>\n";
  ret = ret + t(l + 2) + "<ExecuteAction action=\"" + action_id + "\"/>\n";
  ret = ret + t(l + 1) + "</ReactiveSequence>\n";
  ret = ret + t(l + 1) + "<CheckAtEndReq action=\"" + action_id + "\"/>\n";
  ret = ret + t(l + 1) + "<ApplyAtEndEffect action=\"" + action_id + "\"/>\n";
  ret = ret + t(l) + "</Sequence>\n";
  return ret;
}

std::vector<ActionStamped>
BTBuilder::get_plan_actions(const Plan & plan)
{
  std::vector<ActionStamped> ret;

  for (auto & item : plan) {
    ActionStamped action_stamped;

    action_stamped.time = item.time;
    action_stamped.action = get_action_from_string(item.action, domain_client_);

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
