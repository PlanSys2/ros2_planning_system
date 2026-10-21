// Copyright 2019 Intelligent Robotics Lab
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

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include <optional>
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <set>
#include <map>

#include "plansys2_core/Utils.hpp"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_core/Types.hpp"

namespace plansys2
{

ProblemExpert::ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert)
: domain_expert_(domain_expert)
{
}

bool
ProblemExpert::addInstance(const plansys2::Instance & instance)
{
  plansys2::Instance lowercase_instance = instance;
  std::transform(
    lowercase_instance.type.begin(), lowercase_instance.type.end(), lowercase_instance.type.begin(),
    [](unsigned char c) {return std::tolower(c);});
  for (size_t i = 0; i < lowercase_instance.sub_types.size(); ++i) {
    std::transform(
      lowercase_instance.sub_types[i].begin(), lowercase_instance.sub_types[i].end(),
      lowercase_instance.sub_types[i].begin(), [](unsigned char c) {return std::tolower(c);});
  }

  if (!isValidType(lowercase_instance.type)) {
    return false;
  }

  // Names end up verbatim in the generated problem: they must be PDDL names (#419)
  if (!parser::pddl::isValidName(lowercase_instance.name)) {
    std::cerr << "Invalid instance name [" << lowercase_instance.name << "]" << std::endl;
    return false;
  }

  std::optional<plansys2::Instance> existing_instance = getInstance(lowercase_instance.name);
  bool exist_instance = existing_instance.has_value();

  if (exist_instance && existing_instance.value().type != lowercase_instance.type) {
    return false;
  }

  if (!exist_instance) {
    instances_.push_back(lowercase_instance);
  }

  return true;
}

std::vector<plansys2::Instance>
ProblemExpert::getInstances()
{
  return instances_;
}

bool
ProblemExpert::removeInstance(const plansys2::Instance & instance)
{
  bool found = false;
  size_t i = 0;

  while (!found && i < instances_.size()) {
    if (parser::pddl::compare_str_case_insensitive(instances_[i].name, instance.name)) {
      found = true;
      instances_.erase(instances_.begin() + i);
    }
    i++;
  }

  removeInvalidPredicates(predicates_, instance);
  removeInvalidFunctions(functions_, instance);
  removeInvalidGoals(instance);

  // A metric is dropped in full: removing a term of it would change what is optimized
  if (!metric_.tree.nodes.empty() && !isValidMetric(metric_)) {
    std::cerr << "Metric removed: it refers to the removed instance " << instance.name <<
      std::endl;
    clearMetric();
  }

  // The constraints too: one less would allow what they forbid
  if (!constraints_.nodes.empty() && !isValidConstraints(constraints_)) {
    std::cerr << "Constraints removed: they refer to the removed instance " << instance.name <<
      std::endl;
    clearConstraints();
  }

  return found;
}

std::optional<plansys2::Instance>
ProblemExpert::getInstance(const std::string & instance_name)
{
  plansys2::Instance ret;

  bool found = false;
  size_t i = 0;
  while (i < instances_.size() && !found) {
    if (parser::pddl::compare_str_case_insensitive(instances_[i].name, instance_name)) {
      found = true;
      ret = instances_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<plansys2::Predicate>
ProblemExpert::getPredicates()
{
  // With the derived predicates, to a fixpoint: they can read each other
  return derive(derivedRules(), predicates_, functions_);
}

std::vector<DerivedRule>
ProblemExpert::derivedRules()
{
  std::vector<plansys2::Derived> derived;
  std::set<std::string> names;
  for (const auto & d : domain_expert_->getDerivedPredicates()) {
    if (names.insert(d.predicate.name).second) {
      for (const auto & rule : domain_expert_->getDerivedPredicate(d.predicate.name)) {
        derived.push_back(rule);
      }
    }
  }
  return make_derived_rules(
    derived, instances_,
    [this](const std::string & type) {return domain_expert_->getSubTypes(type);});
}

bool
ProblemExpert::addPredicate(const plansys2::Predicate & predicate)
{
  if (!existPredicate(predicate)) {
    if (isValidPredicate(predicate)) {
      predicates_.push_back(withInstanceTypes(predicate));
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

plansys2_msgs::msg::Node
ProblemExpert::withInstanceTypes(const plansys2_msgs::msg::Node & node)
{
  // Parameters keep the type of their instance, not the one the predicate declares, so a
  // state alone tells the type of each object (what exists and forall range over).
  // Equality of nodes compares names only
  plansys2_msgs::msg::Node ret = node;
  for (auto & param : ret.parameters) {
    auto instance = getInstance(param.name);
    if (instance.has_value()) {
      param.type = instance->type;
    }
  }
  return ret;
}

bool
ProblemExpert::removePredicate(const plansys2::Predicate & predicate)
{
  bool found = false;
  size_t i = 0;

  if (!isValidPredicate(predicate)) {  // if predicate is not valid, error
    return false;
  }
  while (!found && i < predicates_.size()) {
    if (parser::pddl::checkNodeEquality(predicates_[i], predicate)) {
      found = true;
      predicates_.erase(predicates_.begin() + i);
    }
    i++;
  }

  return true;
}

std::optional<plansys2::Predicate>
ProblemExpert::getPredicate(const std::string & expr)
{
  plansys2::Predicate ret;
  plansys2::Predicate pred = parser::pddl::fromStringPredicate(expr);

  bool found = false;
  size_t i = 0;
  while (i < predicates_.size() && !found) {
    if (parser::pddl::checkNodeEquality(predicates_[i], pred)) {
      found = true;
      ret = predicates_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

std::vector<plansys2::Function>
ProblemExpert::getFunctions()
{
  return functions_;
}

bool
ProblemExpert::addFunction(const plansys2::Function & function)
{
  if (!existFunction(function)) {
    if (isValidFunction(function)) {
      functions_.push_back(withInstanceTypes(function));
      return true;
    } else {
      return false;
    }
  } else {
    return updateFunction(function);
  }
}

bool
ProblemExpert::removeFunction(const plansys2::Function & function)
{
  bool found = false;
  size_t i = 0;

  if (!isValidFunction(function)) {  // if function is not valid, error
    return false;
  }
  while (!found && i < functions_.size()) {
    if (parser::pddl::checkNodeEquality(functions_[i], function)) {
      found = true;
      functions_.erase(functions_.begin() + i);
    }
    i++;
  }

  return true;
}

bool
ProblemExpert::updateFunction(const plansys2::Function & function)
{
  if (existFunction(function)) {
    if (isValidFunction(function)) {
      removeFunction(function);
      functions_.push_back(function);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

std::optional<plansys2::Function>
ProblemExpert::getFunction(const std::string & expr)
{
  plansys2::Function ret;
  plansys2::Function func = parser::pddl::fromStringFunction(expr);

  bool found = false;
  size_t i = 0;
  while (i < functions_.size() && !found) {
    if (parser::pddl::checkNodeEquality(functions_[i], func)) {
      found = true;
      ret = functions_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

void
ProblemExpert::removeInvalidPredicates(
  std::vector<plansys2::Predicate> & predicates,
  const plansys2::Instance & instance)
{
  for (auto rit = predicates.rbegin(); rit != predicates.rend(); ++rit) {
    if (std::find_if(
        rit->parameters.begin(), rit->parameters.end(),
        [&](const plansys2_msgs::msg::Param & param) {
          return parser::pddl::compare_str_case_insensitive(param.name, instance.name);
        }) != rit->parameters.end())
    {
      predicates.erase(std::next(rit).base());
    }
  }
}

void
ProblemExpert::removeInvalidFunctions(
  std::vector<plansys2::Function> & functions,
  const plansys2::Instance & instance)
{
  for (auto rit = functions.rbegin(); rit != functions.rend(); ++rit) {
    if (std::find_if(
        rit->parameters.begin(), rit->parameters.end(),
        [&](const plansys2_msgs::msg::Param & param) {
          return parser::pddl::compare_str_case_insensitive(param.name, instance.name);
        }) != rit->parameters.end())
    {
      functions.erase(std::next(rit).base());
    }
  }
}

namespace
{
bool isJunction(const plansys2_msgs::msg::Node & node)
{
  return node.node_type == plansys2_msgs::msg::Node::AND ||
         node.node_type == plansys2_msgs::msg::Node::OR;
}

// Whether the object appears anywhere in this part of the goal
bool mentions(const plansys2_msgs::msg::Tree & tree, uint32_t id, const std::string & name)
{
  const auto & node = tree.nodes[id];
  if (node.node_type == plansys2_msgs::msg::Node::CONSTANT &&
    parser::pddl::compare_str_case_insensitive(node.name, name))
  {
    return true;
  }
  for (const auto & param : node.parameters) {
    if (parser::pddl::compare_str_case_insensitive(param.name, name)) {return true;}
  }
  for (auto child : node.children) {
    if (mentions(tree, child, name)) {return true;}
  }
  return false;
}

// Whether this part goes: and and or, from the root, keep what does not name the object;
// anything else that names it (not, quantifiers, comparisons...) goes as a whole, as pruning
// inside them could make the goal stronger
bool goes(const plansys2_msgs::msg::Tree & tree, uint32_t id, const std::string & name)
{
  const auto & node = tree.nodes[id];
  if (!isJunction(node)) {
    return mentions(tree, id, name);
  }
  if (node.children.empty()) {
    return false;
  }
  for (auto child : node.children) {
    if (!goes(tree, child, name)) {return false;}
  }
  return true;
}

uint32_t copyWithout(
  const plansys2_msgs::msg::Tree & tree, uint32_t id, const std::string & name,
  plansys2_msgs::msg::Tree & out, bool prune = true)
{
  auto node = tree.nodes[id];
  node.node_id = out.nodes.size();
  node.children.clear();
  out.nodes.push_back(node);
  // Only junctions reached through junctions are pruned: the rest is copied as it is
  const bool prune_children = prune && isJunction(node);
  for (auto child : tree.nodes[id].children) {
    if (prune_children && goes(tree, child, name)) {
      continue;
    }
    const auto child_id = copyWithout(tree, child, name, out, prune_children);
    out.nodes[node.node_id].children.push_back(child_id);
  }
  return node.node_id;
}
}  // namespace

void ProblemExpert::removeInvalidGoals(const plansys2::Instance & instance)
{
  if (goal_.nodes.empty()) {
    return;
  }
  // Without the object, what names it can not hold: it goes, the rest of the goal stays
  if (goes(goal_, 0, instance.name)) {
    goal_.nodes.clear();
    return;
  }
  plansys2_msgs::msg::Tree pruned;
  copyWithout(goal_, 0, instance.name, pruned);
  goal_ = plansys2::Goal(pruned);
}

plansys2::Goal
ProblemExpert::getGoal()
{
  return goal_;
}

plansys2::Goal
ProblemExpert::withSubTypes(const plansys2::Goal & goal)
{
  // A goal written as text has the types of its exists and forall variables, but not their
  // subtypes, which say what objects they range over: they come from the domain
  plansys2::Goal ret = goal;
  for (auto & node : ret.nodes) {
    if (node.node_type != plansys2_msgs::msg::Node::EXISTS &&
      node.node_type != plansys2_msgs::msg::Node::FORALL)
    {
      continue;
    }
    for (auto & param : node.parameters) {
      if (!param.type.empty() && param.sub_types.empty()) {
        param.sub_types = domain_expert_->getSubTypes(param.type);
      }
    }
  }
  return ret;
}

bool
ProblemExpert::setGoal(const plansys2::Goal & goal)
{
  if (isValidGoal(goal)) {
    goal_ = withSubTypes(goal);
    return true;
  } else {
    return false;
  }
}

bool ProblemExpert::isGoalSatisfied(const plansys2::Goal & goal)
{
  std::string error;
  if (!parser::pddl::validateTree(goal, error)) {
    std::cerr << "Malformed goal: " << error << std::endl;
    return false;
  }
  // As the clients see it: derived predicates, and quantifiers over all the instances
  return check(withSubTypes(goal), *this);
}

bool
ProblemExpert::clearGoal()
{
  goal_.nodes.clear();
  return true;
}

plansys2::Metric
ProblemExpert::getMetric()
{
  return metric_;
}

bool
ProblemExpert::setMetric(const plansys2::Metric & metric)
{
  if (isValidMetric(metric)) {
    metric_ = metric;
    return true;
  } else {
    return false;
  }
}

bool
ProblemExpert::clearMetric()
{
  metric_ = plansys2::Metric();
  return true;
}

plansys2_msgs::msg::Tree
ProblemExpert::getConstraints()
{
  return constraints_;
}

bool
ProblemExpert::setConstraints(const plansys2_msgs::msg::Tree & constraints)
{
  if (!isValidConstraints(constraints)) {
    return false;
  }
  constraints_ = withSubTypes(constraints);
  return true;
}

bool
ProblemExpert::clearConstraints()
{
  constraints_.nodes.clear();
  return true;
}

bool
ProblemExpert::clearKnowledge()
{
  instances_.clear();
  predicates_.clear();
  functions_.clear();
  clearGoal();
  clearMetric();
  clearConstraints();
  timed_facts_.clear();
  continuous_effects_.clear();

  return true;
}

bool
ProblemExpert::addTimedFact(const plansys2_msgs::msg::Node & fact, double delay, bool negated)
{
  if (fact.node_type == plansys2_msgs::msg::Node::PREDICATE) {
    if (!isValidPredicate(fact)) {
      return false;
    }
  } else if (fact.node_type == plansys2_msgs::msg::Node::FUNCTION) {
    if (negated || !isValidFunction(fact)) {
      return false;
    }
  } else {
    return false;
  }
  TimedFact timed{clock_() + std::max(0.0, delay), negated, fact};
  // In time order; facts at the same time, as given
  auto it = std::upper_bound(
    timed_facts_.begin(), timed_facts_.end(), timed,
    [](const TimedFact & a, const TimedFact & b) {return a.at < b.at;});
  timed_facts_.insert(it, timed);
  return true;
}

std::vector<ProblemExpert::TimedFact>
ProblemExpert::getTimedFacts() const
{
  return timed_facts_;
}

bool
ProblemExpert::setContinuousEffect(
  const std::string & id, const plansys2_msgs::msg::Tree & effects, bool active)
{
  if (!active) {
    continuous_effects_.erase(id);
    return true;
  }
  std::string error;
  if (!effects.nodes.empty() && !parser::pddl::validateTree(effects, error)) {
    std::cerr << "Malformed continuous effects of " << id << ": " << error << std::endl;
    return false;
  }
  continuous_effects_[id] = effects;
  return true;
}

void
ProblemExpert::groundHappenings()
{
  std::string signature;
  for (const auto & instance : instances_) {
    signature += instance.name + ":" + instance.type + " ";
  }
  if (signature == happenings_instances_ && !happenings_instances_.empty()) {
    return;
  }
  happenings_instances_ = signature.empty() ? " " : signature;
  processes_.clear();
  events_.clear();

  // Each process and event for every combination of objects of its parameters' types
  auto ground = [this](
    const std::vector<std::string> & names, bool processes,
    std::vector<plansys2_msgs::msg::Action> & out) {
      for (const auto & name : names) {
        auto model = processes ? domain_expert_->getProcess(name) : domain_expert_->getEvent(name);
        if (!model) {
          continue;
        }
        std::vector<std::vector<std::string>> objects;
        for (const auto & param : model->parameters) {
          const auto subs = domain_expert_->getSubTypes(param.type);
          std::vector<std::string> fit;
          for (const auto & instance : instances_) {
            if (instance.type == param.type || param.type == "object" ||
              std::find(subs.begin(), subs.end(), instance.type) != subs.end())
            {
              fit.push_back(instance.name);
            }
          }
          objects.push_back(fit);
        }
        std::vector<std::vector<std::string>> combinations;
        std::vector<std::string> combination;
        cart_product(combinations, combination, objects.begin(), objects.end());
        for (const auto & values : combinations) {
          auto grounded = processes ? domain_expert_->getProcess(name, values) :
            domain_expert_->getEvent(name, values);
          if (grounded) {
            out.push_back(*grounded);
          }
        }
      }
    };
  ground(domain_expert_->getProcesses(), true, processes_);
  ground(domain_expert_->getEvents(), false, events_);
}

bool
ProblemExpert::stepDynamics(double dt)
{
  groundHappenings();
  bool changed = false;

  // What acts for dt: the running actions, and the processes whose precondition holds
  std::vector<plansys2_msgs::msg::Tree> acting;
  for (const auto & [id, effects] : continuous_effects_) {
    (void)id;
    acting.push_back(effects);
  }
  for (const auto & process : processes_) {
    if (check(process.preconditions, *this)) {
      acting.push_back(process.effects);
    }
  }
  if (dt > 0.0 && !acting.empty()) {
    // Perception measures some: their values stay as it gave them
    std::vector<plansys2::Function> measured;
    for (const auto & function : functions_) {
      if (measured_functions_.count(function.name)) {
        measured.push_back(function);
      }
    }
    for (auto effects : acting) {
      parser::pddl::bindTime(effects, dt);
      plansys2::apply(effects, *this);
    }
    for (const auto & function : measured) {
      updateFunction(function);
    }
    changed = true;
  }

  // Then the events whose precondition holds, each once a step, until none does
  std::set<size_t> fired;
  for (int round = 0; round < 10; round++) {
    bool any = false;
    for (size_t i = 0; i < events_.size(); i++) {
      if (!fired.count(i) && check(events_[i].preconditions, *this)) {
        plansys2::apply(events_[i].effects, *this);
        fired.insert(i);
        any = true;
        std::string event = "event (" + events_[i].name;
        for (const auto & param : events_[i].parameters) {
          event += " " + param.name;
        }
        happened_.push_back(event + ")");
      }
    }
    if (!any) {
      break;
    }
    changed = true;
  }
  return changed;
}

std::vector<std::string>
ProblemExpert::takeHappened()
{
  auto ret = std::move(happened_);
  happened_.clear();
  return ret;
}

bool
ProblemExpert::applyTimedFacts()
{
  const double now = clock_();
  bool applied = false;
  while (!timed_facts_.empty() && timed_facts_.front().at <= now) {
    const auto timed = timed_facts_.front();
    timed_facts_.erase(timed_facts_.begin());
    happened_.push_back(
      std::string("timed fact ") + (timed.negated ? "(not " : "") +
      parser::pddl::toString(timed.fact) +
      (timed.fact.node_type == plansys2_msgs::msg::Node::FUNCTION ?
      " = " + std::to_string(timed.fact.value) : "") + (timed.negated ? ")" : ""));
    if (timed.fact.node_type == plansys2_msgs::msg::Node::PREDICATE) {
      if (timed.negated) {
        removePredicate(timed.fact);
      } else {
        addPredicate(timed.fact);
      }
    } else if (!updateFunction(timed.fact)) {
      addFunction(timed.fact);
    }
    applied = true;
  }
  return applied;
}

bool
ProblemExpert::isValidType(const std::string & type)
{
  std::string lowercase_type = type;
  std::transform(
    lowercase_type.begin(), lowercase_type.end(), lowercase_type.begin(),
    [](unsigned char c) {return std::tolower(c);});

  auto valid_types = domain_expert_->getTypes();
  auto it = std::find(valid_types.begin(), valid_types.end(), lowercase_type);

  return it != valid_types.end();
}

bool
ProblemExpert::existInstance(const std::string & name)
{
  bool found = false;
  size_t i = 0;

  while (!found && i < instances_.size()) {
    if (parser::pddl::compare_str_case_insensitive(instances_[i].name, name)) {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::existPredicate(const plansys2::Predicate & predicate)
{
  bool found = false;
  size_t i = 0;

  while (!found && i < predicates_.size()) {
    if (parser::pddl::checkNodeEquality(predicates_[i], predicate)) {
      found = true;
    }
    i++;
  }

  // A derived one, only if it is
  if (!found && !domain_expert_->getDerivedPredicate(predicate.name).empty()) {
    for (const auto & p : derive(derivedRules(), predicates_, functions_)) {
      if (parser::pddl::checkNodeEquality(p, predicate)) {
        found = true;
        break;
      }
    }
  }

  return found;
}

bool
ProblemExpert::existFunction(const plansys2::Function & function)
{
  bool found = false;
  size_t i = 0;

  while (!found && i < functions_.size()) {
    if (parser::pddl::checkNodeEquality(functions_[i], function)) {
      found = true;
    }
    i++;
  }

  return found;
}

bool
ProblemExpert::argumentsFit(
  const plansys2_msgs::msg::Node & node, const plansys2_msgs::msg::Node & model,
  const std::map<std::string, plansys2_msgs::msg::Param> & bound)
{
  if (model.parameters.size() != node.parameters.size()) {
    return false;
  }

  for (size_t i = 0; i < node.parameters.size(); i++) {
    const auto & expected = model.parameters[i];
    auto fits = [&expected](const std::string & type) {
        return type == expected.type ||
               std::find(expected.sub_types.begin(), expected.sub_types.end(), type) !=
               expected.sub_types.end();
      };

    // A variable of an exists or forall: its type, if it has one, must fit
    auto variable = bound.find(node.parameters[i].name);
    if (variable != bound.end()) {
      if (!variable->second.type.empty() && !fits(variable->second.type)) {
        return false;
      }
      continue;
    }

    auto arg_type = getInstance(node.parameters[i].name);
    if (arg_type.has_value()) {
      if (!fits(arg_type.value().type)) {
        return false;
      }
    } else {
      // It might be a constant of the right type
      auto constants = domain_expert_->getConstants(expected.type);
      if (std::find(constants.begin(), constants.end(), node.parameters[i].name) ==
        constants.end())
      {
        return false;
      }
    }
  }
  return true;
}

bool
ProblemExpert::isValidPredicate(const plansys2::Predicate & predicate)
{
  const std::optional<plansys2::Predicate> & model_predicate =
    domain_expert_->getPredicate(predicate.name);
  return model_predicate && argumentsFit(predicate, model_predicate.value());
}

bool
ProblemExpert::isValidFunction(const plansys2::Function & function)
{
  const std::optional<plansys2::Function> & model_function =
    domain_expert_->getFunction(function.name);
  if (!model_function || !argumentsFit(function, model_function.value())) {
    return false;
  }
  if (function.object.empty()) {
    return true;
  }
  // An object fluent has an object of its type: an instance, or a constant of the domain
  const auto & type = model_function->object;
  if (type.empty()) {
    return false;  // A number, not an object
  }
  const auto sub_types = domain_expert_->getSubTypes(type);
  auto fits = [&](const std::string & t) {
      return t == type || type == "object" ||
             std::find(sub_types.begin(), sub_types.end(), t) != sub_types.end();
    };
  for (const auto & instance : instances_) {
    if (instance.name == function.object) {
      return fits(instance.type);
    }
  }
  for (const auto & constant : domain_expert_->getConstants(type)) {
    if (constant == function.object) {
      return true;
    }
  }
  return false;
}

bool
ProblemExpert::isValidGoal(const plansys2::Goal & goal)
{
  // Goals may come from services: walk them only if they are sound
  std::string error;
  if (!parser::pddl::validateTree(goal, error)) {
    std::cerr << "Malformed goal: " << error << std::endl;
    return false;
  }
  for (const auto & node : goal.nodes) {
    if (node.node_type == plansys2_msgs::msg::Node::CONSTRAINT) {
      std::cerr << "Invalid goal: (" << node.name << ") goes in the constraints" << std::endl;
      return false;
    }
  }
  return checkPredicateTreeTypes(goal, domain_expert_);
}

bool
ProblemExpert::isValidConstraints(const plansys2_msgs::msg::Tree & constraints)
{
  std::string error;
  if (constraints.nodes.empty() || !parser::pddl::validateTree(constraints, error)) {
    std::cerr << "Malformed constraints: " << error << std::endl;
    return false;
  }
  // Modal constraints, in and, forall and preferences; their conditions inside, valid ones
  std::vector<std::pair<uint32_t, bool>> pending = {{0, false}};
  while (!pending.empty()) {
    const auto [id, in_preference] = pending.back();
    pending.pop_back();
    const auto & node = constraints.nodes[id];
    switch (node.node_type) {
      case plansys2_msgs::msg::Node::AND:
      case plansys2_msgs::msg::Node::FORALL:
        for (auto child : node.children) {
          pending.push_back({child, in_preference});
        }
        break;
      case plansys2_msgs::msg::Node::PREFERENCE:
        if (in_preference) {
          std::cerr << "Invalid constraints: a preference in a preference" << std::endl;
          return false;
        }
        pending.push_back({node.children[0], true});
        break;
      case plansys2_msgs::msg::Node::CONSTRAINT:
        for (auto child : node.children) {
          const auto & inner = constraints.nodes[child];
          if (inner.node_type == plansys2_msgs::msg::Node::CONSTRAINT ||
            inner.node_type == plansys2_msgs::msg::Node::PREFERENCE)
          {
            std::cerr << "Invalid constraints: (" << node.name << ") of a constraint" <<
              std::endl;
            return false;
          }
        }
        break;
      default:
        std::cerr << "Invalid constraints: " << parser::pddl::toString(constraints, id) <<
          " is no modal constraint" << std::endl;
        return false;
    }
  }
  return checkPredicateTreeTypes(constraints, domain_expert_);
}

std::set<std::string>
ProblemExpert::getPreferenceNames()
{
  std::set<std::string> ret;
  auto collect = [&ret](const plansys2_msgs::msg::Tree & tree) {
      for (const auto & node : tree.nodes) {
        if (node.node_type == plansys2_msgs::msg::Node::PREFERENCE && !node.name.empty()) {
          ret.insert(node.name);
        }
      }
    };
  collect(goal_);
  collect(constraints_);
  return ret;
}

bool
ProblemExpert::isValidMetric(const plansys2::Metric & metric)
{
  if (metric.optimization != plansys2_msgs::msg::Metric::MINIMIZE &&
    metric.optimization != plansys2_msgs::msg::Metric::MAXIMIZE)
  {
    std::cerr << "Invalid metric optimization: " << static_cast<int>(metric.optimization) <<
      std::endl;
    return false;
  }

  std::string error;
  if (metric.tree.nodes.empty() || !parser::pddl::validateTree(metric.tree, error)) {
    std::cerr << "Malformed metric: " << error << std::endl;
    return false;
  }

  for (const auto & node : metric.tree.nodes) {
    switch (node.node_type) {
      case plansys2_msgs::msg::Node::NUMBER:
        break;
      case plansys2_msgs::msg::Node::EXPRESSION:
        if (node.expression_type != plansys2_msgs::msg::Node::ARITH_ADD &&
          node.expression_type != plansys2_msgs::msg::Node::ARITH_SUB &&
          node.expression_type != plansys2_msgs::msg::Node::ARITH_MULT &&
          node.expression_type != plansys2_msgs::msg::Node::ARITH_DIV)
        {
          std::cerr << "Invalid metric: comparisons are not numeric expressions" << std::endl;
          return false;
        }
        break;
      case plansys2_msgs::msg::Node::FUNCTION:
        if (node.name == "total-time" && node.parameters.empty()) {
          break;
        }
        // PDDL3: how much a preference of the problem is violated
        if (node.name == "is-violated") {
          if (node.parameters.size() != 1 ||
            !getPreferenceNames().count(node.parameters[0].name))
          {
            std::cerr << "Invalid metric: no preference " << parser::pddl::toString(node) <<
              std::endl;
            return false;
          }
          break;
        }
        if (!isValidFunction(node)) {
          std::cerr << "Invalid metric function: " << parser::pddl::toString(node) << std::endl;
          return false;
        }
        break;
      default:
        std::cerr << "Invalid metric: only numbers, functions and arithmetic" << std::endl;
        return false;
    }
  }
  return true;
}

std::string
ProblemExpert::getDomain()
{
  return domain_expert_->getDomain();
}

bool
ProblemExpert::updateDomain(const std::string & domain)
{
  if (!domain_expert_->changeDomain(domain)) {
    return false;
  }

  const bool goal_was_set = !goal_.nodes.empty();

  // 1) Instances whose type no longer exists are removed first: removeInstance()
  // already cascades to any predicate, function or goal subtree referencing them.
  std::vector<plansys2::Instance> stale_instances;
  for (const auto & instance : instances_) {
    if (!isValidType(instance.type)) {
      stale_instances.push_back(instance);
    }
  }
  for (const auto & instance : stale_instances) {
    removeInstance(instance);
  }

  // 2) Predicates/functions whose own signature (name, arity, parameter types) no
  // longer matches the new domain are pruned, even if their instances are fine.
  // Note: removePredicate()/removeFunction() refuse to touch anything that is
  // not valid against the *current* domain_expert_ (already swapped above), so
  // stale entries are erased directly instead of going through them.
  for (auto it = predicates_.begin(); it != predicates_.end(); ) {
    if (!isValidPredicate(*it)) {
      it = predicates_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = functions_.begin(); it != functions_.end(); ) {
    if (!isValidFunction(*it)) {
      it = functions_.erase(it);
    } else {
      ++it;
    }
  }

  // removeInstance() above may already have pruned individual subgoals referencing a
  // removed instance (via removeInvalidGoals()), possibly down to nothing. Beyond
  // that, a goal is only ever dropped in full, never partially: if what remains is
  // invalid for some other reason (e.g. a predicate/function signature change), the
  // whole thing is cleared rather than guessing which subtree to keep, since that
  // would silently change its logical meaning.
  if (goal_was_set && (goal_.nodes.empty() || !isValidGoal(goal_))) {
    clearGoal();
  }

  // The same for the metric and the constraints
  if (!metric_.tree.nodes.empty() && !isValidMetric(metric_)) {
    clearMetric();
  }
  if (!constraints_.nodes.empty() && !isValidConstraints(constraints_)) {
    clearConstraints();
  }

  return true;
}

bool
ProblemExpert::checkPredicateTreeTypes(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<DomainExpert> & domain_expert,
  uint32_t node_id,
  const std::map<std::string, plansys2_msgs::msg::Param> & bound)
{
  if (node_id >= tree.nodes.size()) {
    return false;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id, bound);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::OR: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id, bound);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::NOT: {
        return checkPredicateTreeTypes(tree, domain_expert, tree.nodes[node_id].children[0], bound);
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        auto model = domain_expert->getPredicate(tree.nodes[node_id].name);
        return model && argumentsFit(tree.nodes[node_id], model.value(), bound);
      }

    case plansys2_msgs::msg::Node::FUNCTION: {
        auto model = domain_expert->getFunction(tree.nodes[node_id].name);
        return model && argumentsFit(tree.nodes[node_id], model.value(), bound);
      }

    case plansys2_msgs::msg::Node::EXISTS:
    case plansys2_msgs::msg::Node::FORALL: {
        // The body may use the variables in place of instances
        auto inner = bound;
        for (const auto & variable : tree.nodes[node_id].parameters) {
          inner[variable.name] = variable;
        }
        return checkPredicateTreeTypes(
          tree, domain_expert, tree.nodes[node_id].children[0], inner);
      }

    // PDDL3: what is inside them is checked as any condition; a time is a number
    case plansys2_msgs::msg::Node::PREFERENCE:
    case plansys2_msgs::msg::Node::CONSTRAINT: {
        bool ret = true;
        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id, bound);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::IMPLY: {
        return checkPredicateTreeTypes(
          tree, domain_expert, tree.nodes[node_id].children[0], bound) &&
               checkPredicateTreeTypes(
          tree, domain_expert, tree.nodes[node_id].children[1], bound);
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id, bound);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id, bound);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::NUMBER: {
        return true;
      }

    // An object, compared with an object fluent or another object
    case plansys2_msgs::msg::Node::CONSTANT:
    case plansys2_msgs::msg::Node::PARAMETER: {
        return true;
      }

    default:
      // LCOV_EXCL_START
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        parser::pddl::toString(tree, node_id) << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return false;
}

std::string
ProblemExpert::getProblem()
{
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);

  problem.name = "problem_1";

  for (const auto & instance : instances_) {
    bool is_constant = domain.getType(instance.type)->parseConstant(instance.name).first;
    if (!is_constant) {  // Constants are in the domain
      problem.addObject(instance.name, instance.type);
    }
  }

  for (plansys2_msgs::msg::Node predicate : predicates_) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addInit(predicate.name, v);
  }

  for (plansys2_msgs::msg::Node function : functions_) {
    StringVec v;

    for (size_t i = 0; i < function.parameters.size(); i++) {
      v.push_back(function.parameters[i].name);
    }

    std::transform(
      function.name.begin(), function.name.end(),
      function.name.begin(), ::tolower);

    if (function.object.empty()) {
      problem.addInit(function.name, function.value, v);
    } else {
      problem.addInitObject(function.name, function.object, v);
    }
  }

  // What is still to come, from now
  const double now = clock_();
  for (const auto & timed : timed_facts_) {
    if (timed.at <= now) {
      continue;  // Applied at once
    }
    StringVec v;
    for (const auto & param : timed.fact.parameters) {
      v.push_back(param.name);
    }
    std::string name = timed.fact.name;
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    parser::pddl::TypeGround * ground = nullptr;
    if (timed.fact.node_type == plansys2_msgs::msg::Node::PREDICATE) {
      ground = new parser::pddl::TypeGround(domain.preds.get(name));
    } else {
      ground = new parser::pddl::GroundFunc<double>(domain.funcs.get(name), timed.fact.value);
    }
    ground->insert(domain, v);
    problem.addTimedInit(timed.at - now, timed.negated, ground);
  }

  const std::string gs = parser::pddl::toString(goal_);
  problem.addGoal(gs);

  if (!constraints_.nodes.empty()) {
    try {
      problem.addConstraints(parser::pddl::toString(constraints_));
    } catch (const std::exception & ex) {
      std::cerr << "Constraints left out of the problem: " << ex.what() << std::endl;
    }
  }

  if (!metric_.tree.nodes.empty()) {
    const std::string optimization =
      metric_.optimization == plansys2_msgs::msg::Metric::MAXIMIZE ? "maximize" : "minimize";
    try {
      problem.addMetric(optimization, parser::pddl::toString(metric_.tree));
    } catch (const std::exception & ex) {
      std::cerr << "Metric left out of the problem: " << ex.what() << std::endl;
    }
  }

  std::ostringstream stream;
  stream << problem;
  return stream.str();
}

bool
ProblemExpert::addProblem(const std::string & problem_str)
{
  if (problem_str.find_first_not_of(" \t\r\n") == std::string::npos) {
    std::cerr << "Empty problem." << std::endl;
    return false;
  }

  const int nesting = parser::pddl::getMaxNesting(problem_str);
  if (nesting < 0) {
    std::cerr << "Problem has unbalanced parentheses" << std::endl;
    return false;
  } else if (nesting > parser::pddl::kMaxNestingDepth) {
    std::cerr << "Problem nesting is too deep (" << nesting << " levels)" << std::endl;
    return false;
  }

  // The parser reports malformed input with exceptions: nothing is touched until
  // the whole problem has been parsed
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);
  try {
    std::string lc_problem = problem_str;
    std::transform(
      problem_str.begin(), problem_str.end(), lc_problem.begin(),
      [](unsigned char c) {return std::tolower(c);});

    lc_problem = remove_comments(lc_problem);

    std::string domain_name = problem.getDomainName(lc_problem);
    if (domain_name.empty()) {
      std::cerr << "Domain name is empty" << std::endl;
      return false;
    } else if (!domain_expert_->existDomain(domain_name)) {
      std::cerr << "Domain name does not exist: " << domain_name << std::endl;
      return false;
    }

    domain.name = domain_name;
    problem.parse(lc_problem);
  } catch (const std::exception & ex) {
    std::cerr << "Error parsing problem: " << ex.what() << std::endl;
    return false;
  }

  // Converting the parsed problem can still fail: then nothing of it is kept
  auto prev_instances = instances_;
  auto prev_predicates = predicates_;
  auto prev_functions = functions_;
  auto prev_goal = goal_;
  auto prev_metric = metric_;
  auto prev_constraints = constraints_;
  auto prev_timed_facts = timed_facts_;
  try {
    for (unsigned i = 0; i < domain.types.size(); ++i) {
      if (domain.types[i]->constants.size() ) {
        for (unsigned j = 0; j < domain.types[i]->constants.size(); ++j) {
          plansys2::Instance instance;
          instance.name = domain.types[i]->constants[j];
          instance.type = domain.types[i]->name;
          if (!addInstance(instance)) {
            throw std::runtime_error("Wrong constant " + instance.name);
          }
        }
      }
    }

    for (unsigned i = 0; i < domain.types.size(); ++i) {
      if (domain.types[i]->objects.size() ) {
        for (unsigned j = 0; j < domain.types[i]->objects.size(); ++j) {
          plansys2::Instance instance;
          instance.name = domain.types[i]->objects[j];
          instance.type = domain.types[i]->name;
          if (!addInstance(instance)) {
            throw std::runtime_error("Wrong object " + instance.name);
          }
        }
      }
    }

    plansys2_msgs::msg::Tree tree;
    for (auto ground : problem.init) {
      auto tree_node = ground->getTree(tree, domain);
      switch (tree_node->node_type) {
        case plansys2_msgs::msg::Node::PREDICATE: {
            plansys2::Predicate pred_node(*tree_node);
            if (!addPredicate(pred_node)) {
              throw std::runtime_error(
                      "Wrong predicate " + parser::pddl::toString(tree, tree_node->node_id));
            }
          }
          break;
        case plansys2_msgs::msg::Node::FUNCTION: {
            plansys2::Function func_node(*tree_node);
            if (!addFunction(func_node)) {
              throw std::runtime_error(
                      "Wrong function " + parser::pddl::toString(tree, tree_node->node_id));
            }
          }
          break;
        default:
          break;
      }
    }

    // Timed literals and fluents: from now
    for (const auto & timed : problem.timedInit) {
      plansys2_msgs::msg::Tree fact_tree;
      auto fact = timed.fact->getTree(fact_tree, domain);
      if (!addTimedFact(*fact, timed.time, timed.negated)) {
        throw std::runtime_error(
                "Wrong timed fact " + parser::pddl::toString(fact_tree, fact->node_id));
      }
    }

    // A problem without :goal is valid: the goal can be set later
    if (problem.goal) {
      plansys2_msgs::msg::Tree goal;
      // The objects by name: a function in the goal has them as indexes, (box_pos ?3)
      const std::vector<std::string> objects = problem.ts.tokens;
      problem.goal->getTree(goal, domain, objects);
      if (!setGoal(goal)) {
        throw std::runtime_error("Wrong goal " + parser::pddl::toString(goal));
      }
    }

    // Without :constraints those set before are kept, as the goal; before the metric, that
    // reads their preferences
    if (problem.constraints) {
      plansys2_msgs::msg::Tree constraints;
      problem.constraints->getTree(constraints, domain, problem.ts.tokens);
      if (!setConstraints(constraints)) {
        throw std::runtime_error("Wrong constraints " + parser::pddl::toString(constraints));
      }
    }

    // Without :metric the metric set before is kept, as the goal
    if (problem.metric && problem.metric_expr) {
      std::ostringstream expr;
      problem.metric_expr->PDDLPrint(expr, 0, problem.ts, domain);
      plansys2::Metric metric;
      metric.optimization = problem.maximize ?
        plansys2_msgs::msg::Metric::MAXIMIZE : plansys2_msgs::msg::Metric::MINIMIZE;
      metric.tree = parser::pddl::fromString(
        expr.str(), false, plansys2_msgs::msg::Node::EXPRESSION);
      if (!setMetric(metric)) {
        throw std::runtime_error("Wrong metric " + parser::pddl::toString(metric));
      }
    }
  } catch (const std::exception & ex) {
    std::cerr << "Error adding problem: " << ex.what() << std::endl;
    instances_ = prev_instances;
    predicates_ = prev_predicates;
    functions_ = prev_functions;
    goal_ = prev_goal;
    metric_ = prev_metric;
    constraints_ = prev_constraints;
    timed_facts_ = prev_timed_facts;
    return false;
  }

  return true;
}

}  // namespace plansys2
