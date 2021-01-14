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
      if(req.negative) {
        return false;
      } else {
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

GraphNode::Ptr
BTBuilder::get_node_satisfy(
  const Predicate & predicate,
  const std::list<GraphNode::Ptr> & roots,
  const GraphNode::Ptr & current)
{
  GraphNode::Ptr ret;
  for (const auto & node : roots) {
    auto node_ret  = get_node_satisfy(predicate, node, current);
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
    if (is_action_executable(action, predicates)) {
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
    if (predicates.find({it->toString() , ""}) != predicates.end()) {
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

  // Build the rest of the graph
  while (!action_sequence.empty()) {

    auto new_node = GraphNode::make_shared();
    new_node->action = *action_sequence.begin();

    std::cerr << "Processing " << new_node->action.action->name_actions_to_string() << std::endl;

    std::vector<Predicate> at_start_predicates;
    std::vector<Predicate> over_all_predicates;
    std::vector<Predicate> at_end_predicates;

    action_sequence.begin()->action->at_start_requirements.getPredicates(at_start_predicates);
    action_sequence.begin()->action->over_all_requirements.getPredicates(over_all_predicates);
    action_sequence.begin()->action->at_end_requirements.getPredicates(at_end_predicates);

    auto it_at_start = at_start_predicates.begin();
    while (it_at_start != at_start_predicates.end()) {
      std::cerr << "\t[At Start] Looking for " << it_at_start->toString() << std::endl;
      auto node_satisfy = get_node_satisfy(*it_at_start, graph->roots,new_node);
      if (node_satisfy != nullptr) {
        std::cerr << "\t\tFound in " << node_satisfy->action.action->name << " ";
        for (const auto & param : node_satisfy->action.action->parameters) {
          std::cerr << param.name<< " ";
        }
        std::cerr << std::endl;

        new_node->in_arcs.insert(node_satisfy);
        node_satisfy->out_arcs.insert(new_node);
        it_at_start = at_start_predicates.erase(it_at_start);
      } else {
        ++it_at_start;
      }
    }
    auto it_over_all = over_all_predicates.begin();
    while (it_over_all != over_all_predicates.end()) {
      std::cerr << "\t[Over All] Looking for " << it_over_all->toString() << std::endl;
      auto node_satisfy = get_node_satisfy(*it_over_all, graph->roots, new_node);
      if (node_satisfy != nullptr) {
        std::cerr << "\t\tFound in " << node_satisfy->action.action->name << " ";
        for (const auto & param : node_satisfy->action.action->parameters) {
          std::cerr << param.name<< " ";
        }
        std::cerr << std::endl;

        new_node->in_arcs.insert(node_satisfy);
        node_satisfy->out_arcs.insert(new_node);
        it_over_all =over_all_predicates.erase(it_over_all);
      } else {
        ++it_over_all;
      };
    }

    auto it_at_end = at_end_predicates.begin();
    while (it_at_end != at_end_predicates.end()) {
      std::cerr << "\t[At End] Looking for " << it_at_end->toString() << std::endl;
      auto node_satisfy = get_node_satisfy(*it_at_end, graph->roots, new_node);
      if (node_satisfy != nullptr) {
        std::cerr << "\t\tFound in " << node_satisfy->action.action->name << " ";
        for (const auto & param : node_satisfy->action.action->parameters) {
          std::cerr << param.name<< " ";
        }
        std::cerr << std::endl;
        
        new_node->in_arcs.insert(node_satisfy);
        node_satisfy->out_arcs.insert(new_node);
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

  //is_action_executable()

  return "";
}

/*
  auto levels = get_plan_actions(current_plan);

  // Test the required action for each action
  for (int i = 1; i < levels.size(); i++) {
    int level_comp = i - 1;
    while (level_comp >= 0 && !level_satisfied(levels[i])) {
      check_connections(levels[level_comp], levels[i]);
      level_comp--;
    }
  }

  
  // Remove unnecessary connections
  for (int i = levels.size() - 1; i >= 0; i--) {
    std::cerr << "level: " << levels[i]->time << std::endl;
    for (auto & action_unit : levels[i]->action_units) {
      purge_connections(action_unit);
    }
  }

  // Test if requirement is satisfied by pre-existing knowledge
  for (auto & level : levels) {
    for (auto & action_unit : level->action_units) {
      for (auto & req : action_unit->at_start_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client_->existPredicate(Predicate(req->requirement));
        }
      }
      for (auto & req : action_unit->over_all_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client_->existPredicate(Predicate(req->requirement));
        }
      }
      for (auto & req : action_unit->at_end_reqs) {
        if (!req->satisfied) {
          req->satisfied = problem_client_->existPredicate(Predicate(req->requirement));
        }
      }
    }
  }

  // Check how many independent flows there are
  int root_counters = 0;
  for (auto & level : levels) {
    for (auto & action_unit : level->action_units) {
      if (in_cardinality(action_unit) == 0) {
        root_counters++;
      }
    }
  }

  print_levels(levels);

  std::cerr << "==============> Roots = " << root_counters << std::endl;

  std::string bt_plan;

  if (root_counters > 1) {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n" +
      t(2) + "<Parallel success_threshold=\"" + std::to_string(root_counters) +
      "\" failure_threshold=\"1\">\n";

    for (auto & level : levels) {
      for (auto & action_unit : level->action_units) {
        if (in_cardinality(action_unit) == 0) {
          std::set<ActionUnit::Ptr> used_actions;
          bt_plan = bt_plan + get_flow_tree(action_unit, used_actions, 3);
        }
      }
    }

    bt_plan = bt_plan + t(2) + "</Parallel>\n" +
      t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    bt_plan = std::string("<root main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n";

    for (auto & level : levels) {
      for (auto & action_unit : level->action_units) {
        if (in_cardinality(action_unit) == 0) {
          std::set<ActionUnit::Ptr> used_actions;
          bt_plan = bt_plan + get_flow_tree(action_unit, used_actions, 2);
        }
      }
    }

    bt_plan = bt_plan + t(1) + "</BehaviorTree>\n</root>\n";
  }

  return bt_plan;

  return "";
}*/

std::set<ActionUnit::Ptr>
BTBuilder::pred(ActionUnit::Ptr action_unit)
{
  std::set<ActionUnit::Ptr> deps;
  for (auto & req : action_unit->at_start_reqs) {
    for (auto & effect_con : req->effect_connections) {
      deps.insert(effect_con->action);
    }
  }
  for (auto & req : action_unit->over_all_reqs) {
    for (auto & effect_con : req->effect_connections) {
      deps.insert(effect_con->action);
    }
  }
  for (auto & req : action_unit->at_end_reqs) {
    for (auto & effect_con : req->effect_connections) {
      deps.insert(effect_con->action);
    }
  }

  return deps;
}

std::set<ActionUnit::Ptr>
BTBuilder::succ(ActionUnit::Ptr action_unit)
{
  std::set<ActionUnit::Ptr> deps;
  for (auto & effect : action_unit->at_start_effects) {
    for (auto & req_con : effect->requirement_connections) {
      deps.insert(req_con->action);
    }
  }
  for (auto & effect : action_unit->at_end_effects) {
    for (auto & req_con : effect->requirement_connections) {
      deps.insert(req_con->action);
    }
  }

  return deps;
}

int
BTBuilder::in_cardinality(ActionUnit::Ptr action_unit)
{
  return pred(action_unit).size();
}

int
BTBuilder::out_cardinality(ActionUnit::Ptr action_unit)
{
  return succ(action_unit).size();
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
BTBuilder::get_flow_tree(
  ActionUnit::Ptr root_flow,
  std::set<ActionUnit::Ptr> & used_actions, int level)
{
  std::string ret;
  int l = level;

  used_actions.insert(root_flow);

  if (out_cardinality(root_flow) == 0) {
    if (in_cardinality(root_flow) > 1) {
      ret = t(l) + "<Sequence name=\"" + root_flow->action + ":" +
        std::to_string(root_flow->time) + "\">\n";

      for (auto & action : pred(root_flow)) {
        ret = ret + t(l + 1) + "<WaitAction action=\"" + action->action + ":" +
          std::to_string(action->time) + "\"/>\n";
      }

      ret = ret + execution_block(root_flow->action, root_flow->time, l + 1) +
        t(l) + "</Sequence>\n";
    } else {
      ret = execution_block(root_flow->action, root_flow->time, l);
    }
  }

  if (out_cardinality(root_flow) == 1) {
    ret = t(l) + "<Sequence name=\"" + root_flow->action + ":" +
      std::to_string(root_flow->time) + "\">\n";

    if (in_cardinality(root_flow) > 1) {
      for (auto & action : pred(root_flow)) {
        ret = ret + t(l + 1) + "<WaitAction action=\"" + action->action + ":" +
          std::to_string(action->time) + "\"/>\n";
      }
    }

    ret = ret + execution_block(root_flow->action, root_flow->time, l + 1) +
      get_flow_tree(*succ(root_flow).begin(), used_actions, l + 1) +
      t(l) + "</Sequence>\n";
  }

  if (out_cardinality(root_flow) > 1) {
    ret = t(l) + "<Sequence name=\"" + root_flow->action + ":" +
      std::to_string(root_flow->time) + "\">\n";

    if (in_cardinality(root_flow) > 1) {
      for (auto & action : pred(root_flow)) {
        ret = ret + t(l + 1) + "<WaitAction action=\"" + action->action + ":" +
          std::to_string(action->time) + "\"/>\n";
      }
    }

    ret = ret +
      execution_block(root_flow->action, root_flow->time, l + 1) +
      t(l + 1) + "<Parallel success_threshold=\"" + std::to_string(succ(root_flow).size()) +
      "\"  failure_threshold=\"1\">\n";

    for (auto & action : succ(root_flow)) {
      ret = ret + get_flow_tree(action, used_actions, l + 2);
    }
    ret = ret + t(l + 1) + "</Parallel>\n";
    ret = ret + t(l) + "</Sequence>\n";
  }

  return ret;
}

std::string
BTBuilder::execution_block(const std::string & action, int plan_time, int l)
{
  std::string ret;

  ret = ret + t(l) + "<Sequence name=\"" + action + ":" +
    std::to_string(plan_time) + "\">\n";
  ret = ret + t(l + 1) + "<WaitAtStartReq action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "<ApplyAtStartEffect action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "<Parallel success_threshold=\"2\" failure_threshold=\"1\">\n";
  ret = ret + t(l + 2) + "<CheckOverAllReq action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 2) + "<ExecuteAction action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "</Parallel>\n";
  ret = ret + t(l + 1) + "<CheckAtEndReq action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l + 1) + "<ApplyAtEndEffect action=\"" + action + ":" +
    std::to_string(plan_time) + "\"/>\n";
  ret = ret + t(l) + "</Sequence>\n";

  return ret;
}

void
BTBuilder::check_req_effect(
  std::shared_ptr<plansys2::RequirementConnection> & req,
  std::shared_ptr<plansys2::EffectConnection> & effect)
{
  if (req->requirement == effect->effect) {
    std::cerr << "\tat_start_effects [" << effect->action->action << "] satisfy at_start_req [" << req->requirement << "]" << std::endl;

    req->satisfied = true;
    req->effect_connections.push_back(effect);
    effect->requirement_connections.push_back(req);
  }
}

void
BTBuilder::check_connections(ExecutionLevel::Ptr up_level, ExecutionLevel::Ptr down_level)
{
  for (auto & down_action_unit : down_level->action_units) {
    for (auto & req : down_action_unit->at_start_reqs) {
      if (!req->satisfied) {
        for (auto & up_action_unit : up_level->action_units) {
          for (auto & effect : up_action_unit->at_start_effects) {
            std::cerr << effect->effect << " <-1-> " << req->requirement << std::endl;
            check_req_effect(req, effect);
          }
          for (auto & effect : up_action_unit->at_end_effects) {
            std::cerr << effect->effect << " <-2-> " << req->requirement << std::endl;
            check_req_effect(req, effect);
          }
        }
      }
    }

    for (auto & req : down_action_unit->over_all_reqs) {
      if (!req->satisfied) {
        for (auto & up_action_unit : up_level->action_units) {          
          for (auto & effect : up_action_unit->at_start_effects) {
            std::cerr << effect->effect << " <-3-> " << req->requirement << std::endl;
            check_req_effect(req, effect);
          }
          for (auto & effect : up_action_unit->at_end_effects) {
            std::cerr << effect->effect << " <-4-> " << req->requirement << std::endl;
            check_req_effect(req, effect);
          }
        }
      }
    }

    for (auto & req : down_action_unit->at_end_reqs) {
      if (!req->satisfied) {
        for (auto & up_action_unit : up_level->action_units) {     
          for (auto & effect : up_action_unit->at_start_effects) {
            std::cerr << effect->effect << " <-5-> " << req->requirement << std::endl;
            check_req_effect(req, effect);
          }
          for (auto & effect : up_action_unit->at_end_effects) {
            std::cerr << effect->effect << " <-6-> " << req->requirement << std::endl;
            check_req_effect(req, effect);
          }
        }
      }
    }
  }
}

void
BTBuilder::purge_connections(ActionUnit::Ptr action_unit)
{
  std::cerr << "Purging " << action_unit->action << " =====================================" << std::endl;
  // Get all the requirements
  std::set<RequirementConnection::Ptr> requirements;
  std::copy(
    action_unit->at_start_reqs.begin(),
    action_unit->at_start_reqs.end(),
    std::inserter(requirements, requirements.begin()));
  std::copy(
    action_unit->over_all_reqs.begin(),
    action_unit->over_all_reqs.end(),
    std::inserter(requirements, requirements.begin()));
  std::copy(
    action_unit->at_end_reqs.begin(),
    action_unit->at_end_reqs.end(),
    std::inserter(requirements, requirements.begin()));
  
  for (auto req : requirements) {
    for (auto action_unit_effect : req->effect_connections) {
      std::cerr << "\tPurge " << req->requirement << " in " << action_unit_effect->action->action << std::endl;
      purge_requirement(action_unit_effect->action, requirements);
    }
  }
}

void
BTBuilder::purge_requirement(ActionUnit::Ptr action_unit, std::set<RequirementConnection::Ptr> & requirements_test)
{
  std::cerr << "\t\tPurging with " << action_unit->action << " ===" << std::endl;
  // Get all the requirements of the current action_unit
  std::set<RequirementConnection::Ptr> requirements;
  std::copy(
    action_unit->at_start_reqs.begin(),
    action_unit->at_start_reqs.end(),
    std::inserter(requirements, requirements.begin()));
  std::copy(
    action_unit->over_all_reqs.begin(),
    action_unit->over_all_reqs.end(),
    std::inserter(requirements, requirements.begin()));
  std::copy(
    action_unit->at_end_reqs.begin(),
    action_unit->at_end_reqs.end(),
    std::inserter(requirements, requirements.begin()));

  auto it = requirements_test.begin();
  while (it != requirements_test.end()) {
    for (const auto & req_current_action_unit : requirements) {
      std::cerr << "\t\t\t[" << (*it)->requirement << " - " << req_current_action_unit->requirement << "]" << std::endl;
      if ((*it)->requirement == req_current_action_unit->requirement) {
        std::cerr << "\t\t\t\t****" << (*it)->requirement << " satisfied here" << std::endl;
        
        std::cerr << "Antes " << requirements_test.size() << std::endl;
        it = requirements_test.erase(it);
        std::cerr << "Despues " << requirements_test.size() << std::endl;
      } else {
        ++it;
      }
    }
  }

  // if (!requirements_test.empty()) {
  //   for (auto req : requirements) {
  //     for (auto action_unit_effect : req->effect_connections) {
  //       std::cerr << "\tPurge " << req->requirement << " in " << action_unit_effect->action->action << std::endl;
  //       purge_requirement(action_unit_effect->action, requirements_test);
  //     }
  //   }
  // }
}

bool
BTBuilder::level_satisfied(ExecutionLevel::Ptr level)
{
  bool ret = true;
  for (auto & action_unit : level->action_units) {
    for (auto & req : action_unit->at_start_reqs) {
      ret = ret && req->satisfied;
    }
    for (auto & req : action_unit->over_all_reqs) {
      ret = ret && req->satisfied;
    }
    for (auto & req : action_unit->at_end_reqs) {
      ret = ret && req->satisfied;
    }
  }

  return ret;
}

void
BTBuilder::print_levels(std::vector<ExecutionLevel::Ptr> & levels)
{
  int counter_level = 0;
  for (auto & level : levels) {
    std::cout << "====== Level " << counter_level++ << " [" << level->time << "]" << std::endl;

    for (const auto & action_unit : level->action_units) {
      std::cout << "\t" << action_unit->action << "\tin_cardinality: " <<
        in_cardinality(action_unit) << "\tout_cardinality: " <<
        out_cardinality(action_unit) << std::endl;

      std::cout << "\t\tReqs: " << std::endl;

      // At Start Reqs
      for (const auto & req : action_unit->at_start_reqs) {
        std::cout << "\t\t\tReq At Start: " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;
        
        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }
      for (const auto & req : action_unit->at_start_neg_reqs) {
        std::cout << "\t\t\tReq At Start: NOT " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      // Over All Reqs
      for (const auto & req : action_unit->over_all_reqs) {
        std::cout << "\t\t\tReq Over All: " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      for (const auto & req : action_unit->at_start_neg_reqs) {
        std::cout << "\t\t\tReq Over all: NOT " << req->requirement <<
        (req->satisfied ? " Satisfied" : "  Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      // At End Reqs
      for (const auto & req : action_unit->at_end_reqs) {
        std::cout << "\t\t\tReq At End: " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      for (const auto & req : action_unit->at_start_neg_reqs) {
        std::cout << "\t\t\tReq At End: NOT " << req->requirement <<
        (req->satisfied ? " Satisfied" : " Not satisfied") << std::endl;

        for (auto & action : req->effect_connections) {
          std::cout << "\t\t\t\t" << action->action->action << std::endl;
        }
      }

      std::cout << "\t\tEffects: " << std::endl;

      // At Start Effect
      for (const auto & effect : action_unit->at_start_effects) {
        std::cout << "\t\t\tEffect At Start: " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
      for (const auto & effect : action_unit->at_start_neg_effects) {
        std::cout << "\t\t\tEffect At Start: NOT " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
      // At End Effect
      for (const auto & effect : action_unit->at_end_effects) {
        std::cout << "\t\t\tEffect At End: " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
      for (const auto & effect : action_unit->at_end_neg_effects) {
        std::cout << "\t\t\tEffect At Start: NOT " << effect->effect << std::endl;

        for (auto & req : effect->requirement_connections) {
          std::cout << "\t\t\t\t" << req->action->action << std::endl;
        }
      }
    }
  }
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

bool operator<(const ActionUnit::Ptr & op1, const ActionUnit::Ptr & op2)
{
  std::string op1_str = op1->action + ":" + std::to_string(op1->time);
  std::string op2_str = op2->action + ":" + std::to_string(op2->time);

  return op1_str < op2_str;
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
