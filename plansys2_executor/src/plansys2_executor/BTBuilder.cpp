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

#include "plansys2_executor/BTBuilder.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/Types.hpp"
#include "plansys2_executor/Utils.hpp"

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
  std::set<PredicateStamped> & predicates,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  for (const auto & current_predicate : problem_client->getPredicates()) {
    predicates.insert({current_predicate.toString(), ""});
  }
}

bool
BTBuilder::check_requirements(
  const plansys2::PredicateTree & req_predicates,
  const std::set<PredicateStamped> & predicates) const
{
  std::vector<plansys2::Predicate> reqs;
  req_predicates.getPredicates(reqs);

  for (const auto & req : reqs) {
    bool found = false;
    auto it = predicates.begin();
    while (!found && it != predicates.end()) {
      if (it->predicate == req.toString()) {
        found = true;
      }
      ++it;
    }

    if (found) {
      if (req.negative) {
        return false;
      }
    } else {
      return false;
    }
  }

  return true;
}

bool
BTBuilder::is_action_executable(
  const ActionStamped & action,
  const std::set<PredicateStamped> & predicates) const
{
  return check_requirements(action.action->at_start_requirements, predicates) &&
         check_requirements(action.action->over_all_requirements, predicates) &&
         check_requirements(action.action->at_end_requirements, predicates);
}

void
BTBuilder::apply_action(
  const ActionStamped & action,
  std::set<PredicateStamped> & predicates)
{
  std::vector<Predicate> at_start_effect_predicates;
  action.action->at_start_effects.getPredicates(at_start_effect_predicates);

  for (const auto & effect : at_start_effect_predicates) {
    if (effect.negative) {
      predicates.erase({effect.toString(), ""});
    } else {
      predicates.insert({effect.toString(), ""});
    }
  }

  std::vector<Predicate> at_end_effect_predicates;
  action.action->at_end_effects.getPredicates(at_end_effect_predicates);

  for (const auto & effect : at_end_effect_predicates) {
    if (effect.negative) {
      predicates.erase({effect.toString(), ""});
    } else {
      predicates.insert({effect.toString(), ""});
    }
  }
}


GraphNode::Ptr
BTBuilder::get_node_satisfy(
  const Predicate & predicate,
  const GraphNode::Ptr & node,
  const GraphNode::Ptr & current)
{
  if (node == current) {
    return nullptr;
  }

  GraphNode::Ptr ret = nullptr;
  std::vector<Predicate> at_start_effects;
  std::vector<Predicate> at_end_effects;

  std::vector<Predicate> at_start_requirements;
  std::vector<Predicate> over_all_requirements;
  std::vector<Predicate> at_end_requirements;

  node->action.action->at_start_effects.getPredicates(at_start_effects);
  node->action.action->at_end_effects.getPredicates(at_end_effects);

  node->action.action->at_start_requirements.getPredicates(at_start_requirements);
  node->action.action->over_all_requirements.getPredicates(over_all_requirements);
  node->action.action->at_end_requirements.getPredicates(at_end_requirements);

  for (const auto & effect : at_end_effects) {
    if (effect.toString() == predicate.toString() && effect.negative == predicate.negative) {
      ret = node;
    }
  }

  for (const auto & effect : at_start_effects) {
    if (effect.toString() == predicate.toString() && effect.negative == predicate.negative) {
      ret = node;
    }

    if (effect.toString() == predicate.toString() && effect.negative && !predicate.negative) {
      ret = node;
    }
  }

  for (const auto & req : at_start_requirements) {
    if (req.toString() == predicate.toString() && req.negative == predicate.negative) {
      ret = node;
    }
  }

  for (const auto & req : over_all_requirements) {
    if (req.toString() == predicate.toString() && req.negative == predicate.negative) {
      ret = node;
    }
  }

  for (const auto & req : at_end_requirements) {
    if (req.toString() == predicate.toString() && req.negative == predicate.negative) {
      ret = node;
    }
  }

  for (const auto & arc : node->out_arcs) {
    auto node_ret = get_node_satisfy(predicate, arc, current);

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
  std::vector<Predicate> action_at_start_effects;
  std::vector<Predicate> action_at_end_effects;

  std::vector<Predicate> action_at_start_requirements;
  std::vector<Predicate> action_over_all_requirements;
  std::vector<Predicate> action_at_end_requirements;

  action.action->at_start_effects.getPredicates(action_at_start_effects);
  action.action->at_end_effects.getPredicates(action_at_end_effects);

  action.action->at_start_requirements.getPredicates(action_at_start_requirements);
  action.action->over_all_requirements.getPredicates(action_over_all_requirements);
  action.action->at_end_requirements.getPredicates(action_at_end_requirements);

  for (const auto & other : ret) {
    std::vector<Predicate> other_at_start_effects;
    std::vector<Predicate> other_at_end_effects;

    std::vector<Predicate> other_at_start_requirements;
    std::vector<Predicate> other_over_all_requirements;
    std::vector<Predicate> other_at_end_requirements;

    other->action.action->at_start_effects.getPredicates(other_at_start_effects);
    other->action.action->at_end_effects.getPredicates(other_at_end_effects);

    other->action.action->at_start_requirements.getPredicates(other_at_start_requirements);
    other->action.action->over_all_requirements.getPredicates(other_over_all_requirements);
    other->action.action->at_end_requirements.getPredicates(other_at_end_requirements);

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
  const Predicate & predicate,
  const std::list<GraphNode::Ptr> & roots,
  const GraphNode::Ptr & current)
{
  GraphNode::Ptr ret;
  for (const auto & node : roots) {
    auto node_ret = get_node_satisfy(predicate, node, current);
    if (node_ret != nullptr) {
      ret = node_ret;
    }
  }

  return ret;
}

std::list<GraphNode::Ptr>
BTBuilder::get_roots(
  std::vector<plansys2::ActionStamped> & action_sequence,
  const std::set<PredicateStamped> & predicates)
{
  std::list<GraphNode::Ptr> ret;

  auto it = action_sequence.begin();
  while (it != action_sequence.end()) {
    const auto & action = *it;
    if (is_action_executable(action, predicates) && is_parallelizable(action, ret)) {
      auto new_root = GraphNode::make_shared();
      new_root->action = action;

      ret.push_back(new_root);
      it = action_sequence.erase(it);
    } else {
      break;
    }
  }

  return ret;
}

void
BTBuilder::remove_existing_predicates(
  std::vector<Predicate> & check_predicates,
  const std::set<PredicateStamped> & predicates) const
{
  auto it = check_predicates.begin();
  while (it != check_predicates.end()) {
    if (predicates.find({it->toString(), ""}) != predicates.end()) {
      it = check_predicates.erase(it);
    } else {
      ++it;
    }
  }
}

Graph::Ptr
BTBuilder::get_graph(const Plan & current_plan)
{
  std::set<PredicateStamped> predicates;
  auto graph = Graph::make_shared();

  auto action_sequence = get_plan_actions(current_plan);
  init_predicates(predicates, problem_client_);

  graph->roots = get_roots(action_sequence, predicates);

  // Apply roots actions
  for (auto & action_node : graph->roots) {
    apply_action(action_node->action, predicates);
  }

  std::set<plansys2::GraphNode::Ptr> used_nodes;

  // Build the rest of the graph
  while (!action_sequence.empty()) {
    auto new_node = GraphNode::make_shared();
    new_node->action = *action_sequence.begin();

    std::vector<Predicate> at_start_predicates;
    std::vector<Predicate> over_all_predicates;
    std::vector<Predicate> at_end_predicates;

    action_sequence.begin()->action->at_start_requirements.getPredicates(at_start_predicates);
    action_sequence.begin()->action->over_all_requirements.getPredicates(over_all_predicates);
    action_sequence.begin()->action->at_end_requirements.getPredicates(at_end_predicates);

    auto it_at_start = at_start_predicates.begin();
    while (it_at_start != at_start_predicates.end()) {
      auto node_satisfy = get_node_satisfy(*it_at_start, graph->roots, new_node);
      if (node_satisfy != nullptr) {
        if (used_nodes.find(new_node) == used_nodes.end()) {
          new_node->in_arcs.insert(node_satisfy);
          node_satisfy->out_arcs.insert(new_node);
          used_nodes.insert(new_node);
        }

        it_at_start = at_start_predicates.erase(it_at_start);
      } else {
        ++it_at_start;
      }
    }
    auto it_over_all = over_all_predicates.begin();
    while (it_over_all != over_all_predicates.end()) {
      auto node_satisfy = get_node_satisfy(*it_over_all, graph->roots, new_node);
      if (node_satisfy != nullptr) {
        if (used_nodes.find(new_node) == used_nodes.end()) {
          new_node->in_arcs.insert(node_satisfy);
          node_satisfy->out_arcs.insert(new_node);
          used_nodes.insert(new_node);
        }

        it_over_all = over_all_predicates.erase(it_over_all);
      } else {
        ++it_over_all;
      }
    }

    auto it_at_end = at_end_predicates.begin();
    while (it_at_end != at_end_predicates.end()) {
      auto node_satisfy = get_node_satisfy(*it_at_end, graph->roots, new_node);
      if (node_satisfy != nullptr) {
        if (used_nodes.find(new_node) == used_nodes.end()) {
          new_node->in_arcs.insert(node_satisfy);
          node_satisfy->out_arcs.insert(new_node);
          used_nodes.insert(new_node);
        }

        it_at_end = at_end_predicates.erase(it_at_end);
      } else {
        ++it_at_end;
      }
    }

    remove_existing_predicates(at_start_predicates, predicates);
    remove_existing_predicates(over_all_predicates, predicates);
    remove_existing_predicates(at_end_predicates, predicates);

    assert(at_start_predicates.empty());
    assert(over_all_predicates.empty());
    assert(at_end_predicates.empty());

    action_sequence.erase(action_sequence.begin());
  }

  return graph;
}

std::string
BTBuilder::get_tree(const Plan & current_plan)
{
  auto action_graph = get_graph(current_plan);

  std::string bt_plan;

  if (action_graph->roots.size() > 1) {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n" +
      t(2) + "<Parallel success_threshold=\"" + std::to_string(action_graph->roots.size()) +
      "\" failure_threshold=\"1\">\n";

    for (const auto & node : action_graph->roots) {
      bt_plan = bt_plan + get_flow_tree(node, 3);
    }

    bt_plan = bt_plan + t(2) + "</Parallel>\n" +
      t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n";

    bt_plan = bt_plan + get_flow_tree(*action_graph->roots.begin(), 2);

    bt_plan = bt_plan + t(1) + "</BehaviorTree>\n</root>\n";
  }

  return bt_plan;
}

std::string
BTBuilder::get_flow_tree(
  GraphNode::Ptr node,
  int level)
{
  std::string ret;
  int l = level;

  const std::string action_id = "(" + node->action.action->name_actions_to_string() + "):" +
    std::to_string(static_cast<int>(node->action.time * 1000));

  if (node->out_arcs.size() == 0) {
    ret = ret + execution_block(node, l);
  } else if (node->out_arcs.size() == 1) {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    for (const auto & child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, l + 1);
    }

    ret = ret + t(l) + "</Sequence>\n";
  } else {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    ret = ret + t(l + 1) +
      "<Parallel success_threshold=\"" + std::to_string(node->out_arcs.size()) +
      "\" failure_threshold=\"1\">\n";

    for (const auto & child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, l + 2);
    }

    ret = ret + t(l + 1) + "</Parallel>\n";
    ret = ret + t(l) + "</Sequence>\n";
  }

  return ret;
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
  ret = ret + t(l + 1) + "<Parallel success_threshold=\"2\" failure_threshold=\"1\">\n";
  ret = ret + t(l + 2) + "<CheckOverAllReq action=\"" + action_id + "\"/>\n";
  ret = ret + t(l + 2) + "<ExecuteAction action=\"" + action_id + "\"/>\n";
  ret = ret + t(l + 1) + "</Parallel>\n";
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
  std::cerr << " in arcs " << node->in_arcs.size() << std::endl;

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


bool operator<(const PredicateStamped & op1, const PredicateStamped & op2)
{
  return op1.predicate < op2.predicate;
}

bool operator<(const PredicateStamped & op1, const Predicate & op2)
{
  return op1.predicate < op2.toString();
}

}  // namespace plansys2
