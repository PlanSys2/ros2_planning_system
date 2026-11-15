// Copyright 2026 Juan Carlos Manzanares Serrano
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

#include "plansys2_state_updater/domain_check.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_state_updater/engine.hpp"
#include "plansys2_state_updater/instances.hpp"
#include "plansys2_state_updater/rules.hpp"

namespace plansys2_state_updater
{

using plansys2_msgs::msg::Node;

std::pair<std::set<std::string>, std::set<std::string>>
modified_by(const std::vector<plansys2_msgs::msg::Tree> & effects)
{
  std::set<std::string> predicates, functions;
  for (const auto & tree : effects) {
    if (tree.nodes.empty()) {
      continue;
    }
    std::vector<uint32_t> pending = {0};
    while (!pending.empty()) {
      const auto & node = tree.nodes[pending.back()];
      pending.pop_back();
      std::vector<uint32_t> children(node.children.begin(), node.children.end());
      if (node.node_type == Node::PREDICATE) {
        predicates.insert(node.name);
      } else if (node.node_type == Node::FUNCTION_MODIFIER) {
        if (!children.empty() && tree.nodes[children[0]].node_type == Node::FUNCTION) {
          functions.insert(tree.nodes[children[0]].name);
        }
        continue;  // Its other operand is only read
      } else if (node.node_type == Node::WHEN && !children.empty()) {
        children.erase(children.begin());  // Its condition is only read
      }
      pending.insert(pending.end(), children.begin(), children.end());
    }
  }
  return {predicates, functions};
}

namespace
{

enum class TypeProblem {NONE, MISSING, TYPE};

TypeProblem
type_problem(
  const std::string & object, const std::string & type, const std::vector<std::string> & subtypes,
  const std::optional<std::map<std::string, std::string>> & objects)
{
  if (!objects || type.empty() || type == "object") {
    return TypeProblem::NONE;
  }
  auto it = objects->find(object);
  if (it == objects->end()) {
    return TypeProblem::MISSING;
  }
  if (it->second != type &&
    std::find(subtypes.begin(), subtypes.end(), it->second) == subtypes.end())
  {
    return TypeProblem::TYPE;
  }
  return TypeProblem::NONE;
}

// The objects of a parameter: a name, or the values of an enum
void
check_object(
  const std::string & what, const std::string & object, const std::string & type,
  const std::vector<std::string> & subtypes, const DomainInfo & domain,
  std::vector<std::string> & errors, std::vector<std::string> & warnings)
{
  switch (type_problem(object, type, subtypes, domain.objects)) {
    case TypeProblem::TYPE:
      errors.push_back(
        what + ": '" + object + "' is a " + domain.objects->at(object) +
        ", the parameter is a " + type);
      break;
    case TypeProblem::MISSING:
      warnings.push_back(
        what + ": '" + object + "' is not in the problem yet: its writes fail until it is");
      break;
    default:
      break;
  }
}

// The fact of a template against its signature; false if it does not even fit it
bool
check_signature(
  const std::string & rule, const std::string & kind, const Template & fact_template,
  const DomainInfo & domain, std::vector<std::string> & errors)
{
  const auto & known = kind == "function" ? domain.functions : domain.predicates;
  auto it = known.find(fact_template.name());
  if (it == known.end()) {
    errors.push_back(
      "rule '" + rule + "': " + kind + " '" + fact_template.name() + "' is not in the domain");
    return false;
  }
  if (it->second.size() != fact_template.params().size()) {
    errors.push_back(
      "rule '" + rule + "': '" + fact_template.fact() + "' has " +
      std::to_string(fact_template.params().size()) + " parameters, the domain's " +
      fact_template.name() + " has " + std::to_string(it->second.size()));
    return false;
  }
  return true;
}

void
check_rule(
  const Rule & rule, const DomainInfo & domain, std::vector<std::string> & errors,
  std::vector<std::string> & warnings)
{
  struct Checked
  {
    std::string kind;
    const Template * fact_template;
    std::vector<std::string> enum_values;
  };
  std::vector<Checked> facts = {{rule.kind, &*rule.fact_template, rule.enum_values}};
  if (rule.stale_template) {
    facts.push_back({"predicate", &*rule.stale_template, {}});
  }
  for (const auto & [kind, fact_template, enum_values] : facts) {
    if (!check_signature(rule.name, kind, *fact_template, domain, errors)) {
      continue;
    }
    const std::string what = "rule '" + rule.name + "': " + kind + " '" +
      fact_template->name() + "'";
    const auto & signature = (kind == "function" ? domain.functions : domain.predicates).at(
      fact_template->name());
    for (size_t i = 0; i < signature.size(); i++) {
      const auto & param = fact_template->params()[i];
      const auto & [type, subtypes] = signature[i];
      if (std::holds_alternative<std::string>(param)) {
        check_object(
          what, std::get<std::string>(param), type, subtypes, domain, errors, warnings);
      } else if (std::get<Path>(param).empty()) {
        for (const auto & value : enum_values) {
          check_object(what, value, type, subtypes, domain, errors, warnings);
        }
      }
    }
    const auto & modified = kind == "function" ? domain.modified_functions :
      domain.modified_predicates;
    if (modified.count(fact_template->name())) {
      const bool measured = domain.measured_functions &&
        domain.measured_functions->count(fact_template->name());
      if (kind == "predicate") {
        errors.push_back(
          what + " is changed by the actions of the plan: the state updater would fight the "
          "executor");
      } else if (rule.mode != "correct" && !measured) {
        errors.push_back(
          what + " is changed by the actions of the plan: it needs mode 'correct' (or to be "
          "in measured_functions)");
      }
    }
  }
  // An object fluent has an object, not a number: no model moves it, no change is small
  auto fluent = domain.object_fluents.find(rule.fact_name);
  if (rule.kind == "function" && fluent != domain.object_fluents.end()) {
    const std::string what = "rule '" + rule.name + "': function '" + rule.fact_name + "'";
    if (rule.mode == "correct") {
      errors.push_back(what + " has an object, not a number: mode 'correct' is for numbers");
    }
    if (rule.min_change) {
      errors.push_back(what + " has an object, not a number: min_change is for numbers");
    }
    if (rule.has_stale_value) {
      if (!rule.stale_value.is_string()) {
        errors.push_back(what + " has an object: its on_stale value is one, not a number");
      } else {
        check_object(
          what, rule.stale_value.as_string(), fluent->second, {}, domain, errors, warnings);
      }
    }
  } else if (rule.kind == "function" && rule.has_stale_value && rule.stale_value.is_string()) {
    errors.push_back(
      "rule '" + rule.name + "': function '" + rule.fact_name + "' has a number: its on_stale "
      "value is a number");
  }
  if (domain.measured_functions && rule.kind == "function" && rule.mode == "correct" &&
    domain.measured_functions->count(rule.fact_name))
  {
    errors.push_back(
      "rule '" + rule.name + "': mode 'correct' on '" + rule.fact_name + "', which is in "
      "measured_functions of problem_expert: no model to correct");
  }
}

void
check_instance_rule(
  const InstanceRule & rule, const DomainInfo & domain, std::vector<std::string> & errors,
  std::vector<std::string> & warnings)
{
  // A type of the message ({class}) is known as each object comes: checked then
  const bool fixed_type = !rule.type_template || !rule.type_template->is_template();
  if (fixed_type && domain.types && !domain.types->count(rule.type)) {
    errors.push_back("rule '" + rule.name + "': the type '" + rule.type + "' is not in the domain");
  }
  const Path self = {PathStep{PathStep::FIELD, "self", 0}};
  for (const auto & fact_template : rule.fact_templates) {
    if (!check_signature(rule.name, "predicate", fact_template, domain, errors)) {
      continue;
    }
    const std::string what = "rule '" + rule.name + "': predicate '" + fact_template.name() + "'";
    const auto & signature = domain.predicates.at(fact_template.name());
    for (size_t i = 0; i < signature.size(); i++) {
      const auto & param = fact_template.params()[i];
      const auto & [type, subtypes] = signature[i];
      if (std::holds_alternative<Path>(param) && std::get<Path>(param) == self) {
        if (fixed_type && !type.empty() && type != "object" && rule.type != type &&
          std::find(subtypes.begin(), subtypes.end(), rule.type) == subtypes.end())
        {
          errors.push_back(
            what + ": its objects are " + rule.type + ", the parameter is a " + type);
        }
      } else if (std::holds_alternative<std::string>(param)) {
        check_object(
          what, std::get<std::string>(param), type, subtypes, domain, errors, warnings);
      }
    }
    if (domain.modified_predicates.count(fact_template.name())) {
      errors.push_back(
        what + " is changed by the actions of the plan: the state updater would fight the "
        "executor");
    }
  }
}

}  // namespace

std::pair<std::vector<std::string>, std::vector<std::string>>
check_domain(const Engine & engine, const DomainInfo & domain)
{
  std::vector<std::string> errors, warnings;
  for (const auto & rule : engine.rules()) {
    warnings.insert(warnings.end(), rule->warnings.begin(), rule->warnings.end());
    if (const auto * instances = dynamic_cast<const InstanceRule *>(rule.get())) {
      check_instance_rule(*instances, domain, errors, warnings);
    } else {
      check_rule(dynamic_cast<const Rule &>(*rule), domain, errors, warnings);
    }
  }
  return {errors, warnings};
}

}  // namespace plansys2_state_updater
