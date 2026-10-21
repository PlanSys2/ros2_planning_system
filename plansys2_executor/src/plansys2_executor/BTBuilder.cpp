// Copyright 2026 Intelligent Robotics Lab
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

#include "plansys2_executor/BTBuilder.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"

namespace plansys2
{

void
BTBuilder::set_clients(
  std::shared_ptr<plansys2::DomainExpertClient> domain_client,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  domain_client_ = domain_client;
  problem_client_ = problem_client;
}

void
BTBuilder::set_initial_state(
  std::vector<plansys2::Predicate> predicates, std::vector<plansys2::Function> functions)
{
  initial_state_ = std::make_pair(std::move(predicates), std::move(functions));
}

std::vector<plansys2::Predicate>
BTBuilder::initial_predicates() const
{
  if (initial_state_) {
    return initial_state_->first;
  }
  // Read once per plan: the builders simulate the plan from it many times
  if (!current_state_) {
    current_state_ = std::make_pair(
      problem_client()->getPredicates(), problem_client()->getFunctions());
  }
  return current_state_->first;
}

std::vector<plansys2::Function>
BTBuilder::initial_functions() const
{
  if (initial_state_) {
    return initial_state_->second;
  }
  initial_predicates();
  return current_state_->second;
}

plansys2::ActionVariant
BTBuilder::plan_item_action(
  plansys2::DomainExpertClient & domain_client, const std::vector<std::string> & actions,
  const plansys2_msgs::msg::PlanItem & item)
{
  const auto name = get_action_name(item.action);
  const auto params = get_action_params(item.action);
  plansys2::ActionVariant ret;
  if (std::find(actions.begin(), actions.end(), name) != actions.end()) {
    ret = domain_client.getAction(name, params);
    return ret;
  }
  auto action = domain_client.getDurativeAction(name, params);
  if (action) {
    // The duration the plan chose, wherever the action reads it
    for (auto * tree : {&action->at_start_requirements, &action->over_all_requirements,
        &action->at_end_requirements, &action->at_start_effects, &action->at_end_effects,
        &action->duration, &action->continuous_effects})
    {
      parser::pddl::bindDuration(*tree, item.duration);
    }
  }
  ret = action;
  return ret;
}

std::vector<ActionStamped>
BTBuilder::timed_events() const
{
  if (timed_events_) {
    return *timed_events_;
  }
  timed_events_.emplace();
  try {
    parser::pddl::Domain domain(domain_client()->getDomain());
    parser::pddl::Instance problem(domain);
    problem.parse(problem_client()->getProblem());
    for (const auto & timed : problem.timedInit) {
      plansys2_msgs::msg::Tree fact_tree;
      auto fact = timed.fact->getTree(fact_tree, domain);
      const auto text = parser::pddl::toString(fact_tree, fact->node_id);
      std::string effect;
      if (fact->node_type == plansys2_msgs::msg::Node::FUNCTION) {
        // Its value from then on
        std::string function = "(" + fact->name;
        for (const auto & param : fact->parameters) {
          function += " " + param.name;
        }
        effect = "(and (assign " + function + ") " + std::to_string(fact->value) + "))";
      } else {
        effect = timed.negated ? "(and (not " + text + "))" : "(and " + text + ")";
      }
      auto action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
      action->name = "timed";
      action->at_end_effects = parser::pddl::fromString(effect);
      ActionStamped event;
      event.time = timed.time + time_offset_;  // In the time of the plan
      event.duration = 0.0;
      event.expression = "(timed " + std::to_string(timed.time) + " " + effect + ")";
      event.type = ActionType::TIMED;
      event.action = action;
      timed_events_->push_back(event);
    }
  } catch (const std::exception & e) {
    std::cerr << "timed_events: the problem cannot be read: " << e.what() << std::endl;
  }
  return *timed_events_;
}

void
BTBuilder::collect_dynamic() const
{
  if (dynamic_names_) {
    return;
  }
  // What the effects of processes, events and continuous effects change, by name
  dynamic_names_.emplace();
  dynamic_functions_.emplace();
  auto collect = [this](const plansys2_msgs::msg::Tree & effects) {
      for (const auto & node : effects.nodes) {
        if (node.node_type == plansys2_msgs::msg::Node::FUNCTION_MODIFIER &&
          !node.children.empty())
        {
          dynamic_names_->insert(effects.nodes[node.children[0]].name);
          dynamic_functions_->insert(effects.nodes[node.children[0]].name);
        } else if (node.node_type == plansys2_msgs::msg::Node::PREDICATE) {
          dynamic_names_->insert(node.name);
        }
      }
    };
  for (const auto & name : domain_client()->getProcesses()) {
    if (auto process = domain_client()->getProcess(name)) {
      collect(process->effects);
    }
  }
  for (const auto & name : domain_client()->getEvents()) {
    if (auto event = domain_client()->getEvent(name)) {
      collect(event->effects);
    }
  }
  for (const auto & name : domain_client()->getDurativeActions()) {
    if (auto action = domain_client()->getDurativeAction(name)) {
      collect(action->continuous_effects);
    }
  }
}

bool
BTBuilder::reads_dynamic(const plansys2_msgs::msg::Tree & conditions) const
{
  collect_dynamic();
  if (dynamic_names_->empty()) {
    return false;
  }
  for (const auto & node : conditions.nodes) {
    if ((node.node_type == plansys2_msgs::msg::Node::PREDICATE ||
      node.node_type == plansys2_msgs::msg::Node::FUNCTION) && dynamic_names_->count(node.name))
    {
      return true;
    }
  }
  return false;
}

plansys2_msgs::msg::Tree
BTBuilder::relax_dynamic(const plansys2_msgs::msg::Tree & conditions) const
{
  collect_dynamic();
  if (dynamic_functions_->empty()) {
    return conditions;
  }
  auto reads = [&](uint32_t root) {
      std::vector<uint32_t> pending{root};
      while (!pending.empty()) {
        const auto & node = conditions.nodes[pending.back()];
        pending.pop_back();
        if (node.node_type == plansys2_msgs::msg::Node::FUNCTION &&
          dynamic_functions_->count(node.name))
        {
          return true;
        }
        pending.insert(pending.end(), node.children.begin(), node.children.end());
      }
      return false;
    };
  auto ret = conditions;
  for (uint32_t i = 0; i < ret.nodes.size(); i++) {
    auto & node = ret.nodes[i];
    if (node.node_type == plansys2_msgs::msg::Node::EXPRESSION && reads(i)) {
      // An empty and holds
      node.node_type = plansys2_msgs::msg::Node::AND;
      node.children.clear();
      node.negate = false;
    }
  }
  return ret;
}

void
BTBuilder::apply_effects(
  const plansys2_msgs::msg::Tree & effects, std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  plansys2::apply(effects, predicates, functions);
  load_derived();
  if (!derived_rules_->empty()) {
    predicates = derive(*derived_rules_, predicates, functions);
  }
}

void
BTBuilder::load_derived() const
{
  // Read once per plan; a domain without derived predicates pays nothing more
  if (derived_rules_) {
    return;
  }
  std::vector<plansys2::Derived> derived;
  derived_names_.emplace();
  for (const auto & d : domain_client()->getDerivedPredicates()) {
    if (derived_names_->insert(d.predicate.name).second) {
      for (const auto & rule : domain_client()->getDerivedPredicate(d.predicate.name)) {
        derived.push_back(rule);
      }
    }
  }
  derived_rules_ = std::make_shared<const std::vector<plansys2::DerivedRule>>(
    derived.empty() ? std::vector<plansys2::DerivedRule>{} :
    make_derived_rules(derived, problem_client()->getInstances()));
}

const std::set<std::string> &
BTBuilder::derived_names() const
{
  load_derived();
  return *derived_names_;
}

std::shared_ptr<plansys2::DomainExpertClient>
BTBuilder::domain_client() const
{
  // The clients of its executor, on the executor's node: a builder makes no node
  if (!domain_client_) {
    throw std::logic_error("BTBuilder without clients: call set_clients first");
  }
  return domain_client_;
}

std::shared_ptr<plansys2::ProblemExpertClient>
BTBuilder::problem_client() const
{
  if (!problem_client_) {
    throw std::logic_error("BTBuilder without clients: call set_clients first");
  }
  return problem_client_;
}

}  // namespace plansys2
