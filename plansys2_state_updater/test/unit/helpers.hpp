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

// Helpers shared by the tests of the core

#ifndef UNIT__HELPERS_HPP_
#define UNIT__HELPERS_HPP_

#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "plansys2_state_updater/engine.hpp"

namespace plansys2_state_updater
{

// So that gtest prints the values in its failures
inline void PrintTo(const FactValue & value, std::ostream * os)
{
  *os << to_text(value);
}

}  // namespace plansys2_state_updater

namespace su = plansys2_state_updater;

using Facts = std::vector<std::pair<std::string, su::FactValue>>;
using Kinds = std::vector<std::tuple<std::string, std::string, su::FactValue>>;

inline su::Value msg(const std::string & yaml)
{
  return su::from_yaml(YAML::Load(yaml));
}

// A string literal would convert to bool, not to std::string
inline su::FactValue S(const char * text)
{
  return su::FactValue(std::string(text));
}

inline std::unique_ptr<su::Engine> engine(
  const std::vector<std::string> & rules, su::Engine::CurrentValue current_value = {},
  double now = 0.0, su::Engine::InUse in_use = {})
{
  std::string yaml = "state_updater:\n  rules:\n";
  for (const auto & rule : rules) {
    yaml += "    - " + rule + "\n";
  }
  auto e = su::Engine::from_config(YAML::Load(yaml), current_value, in_use);
  e->start(now);
  return e;
}

inline std::string config_error(const std::vector<std::string> & rules)
{
  try {
    engine(rules);
  } catch (const su::ConfigError & e) {
    return e.what();
  }
  return "";
}

inline Facts facts(const std::vector<su::Change> & changes)
{
  Facts out;
  for (const auto & change : changes) {
    out.emplace_back(change.fact, change.value);
  }
  return out;
}

inline Kinds kinds(const std::vector<su::Change> & changes)
{
  Kinds out;
  for (const auto & change : changes) {
    out.emplace_back(change.kind, change.fact, change.value);
  }
  return out;
}

inline bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

inline const su::Value::Map & status(const su::Engine & e, size_t rule)
{
  static su::Value last;
  last = e.status();
  return last.as_list()[rule].as_map();
}

inline std::string battery(const std::string & policy = "{}")
{
  return "{name: battery, topic: /battery, field: percentage, "
         "transform: {linear: {scale: 100.0}}, target: {function: '(energy)'}, policy: " +
         policy + "}";
}

inline std::string leak(const std::string & policy = "{}")
{
  return "{name: leak, topic: /leak, field: data, target: {predicate: '(leak_detected)'}, "
         "policy: " + policy + "}";
}

inline std::string diagnostics(const std::string & policy = "{}")
{
  return "{name: sensors, topic: /diagnostics, field: 'status[*]', "
         "transform: {compare: {field: level, op: '==', value: 0}}, "
         "target: {predicate: '(sensor_ok {name})'}, policy: " + policy + "}";
}

inline std::string reachable(
  const std::string & policy = "{}", const std::string & field = "waypoints",
  const std::string & fact = "(reachable {})")
{
  return "{name: reachable, topic: /reachable, field: '" + field + "', target: {set: '" + fact +
         "'}, policy: " + policy + "}";
}

inline std::string mode(const std::string & policy = "{}", bool with_values = true)
{
  return "{name: mode, topic: /mode, field: mode, "
         "transform: {map: {values: {2: normal, 3: degraded}, default: lost}}, "
         "target: {enum: '(mode_is {})'" +
         std::string(with_values ? ", values: [normal, degraded, lost]" : "") +
         "}, policy: " + policy + "}";
}

inline std::string contacts(const std::string & policy = "{expire_after: 10.0}")
{
  return "{name: contacts, topic: /sonar, field: 'contacts[*]', "
         "target: {instance: {name: 'contact_{id}', type: contact}, "
         "facts: ['(contact_at {self} {waypoint})', '(detected {self})']}, policy: " +
         policy + "}";
}

inline su::ProblemState problem(
  const std::set<std::string> & predicates, const std::map<std::string, double> & functions = {},
  std::optional<std::set<std::string>> instances = std::nullopt,
  const std::map<std::string, std::string> & objects = {})
{
  su::ProblemState state;
  state.predicates = predicates;
  state.functions = functions;
  state.objects = objects;
  state.instances = instances;
  return state;
}

inline su::Value status_msg(const std::vector<std::pair<std::string, int>> & entries)
{
  su::Value::List list;
  for (const auto & [name, level] : entries) {
    list.push_back(su::Value(su::Value::Map{{"name", su::Value(name)}, {"level", level}}));
  }
  return su::Value(su::Value::Map{{"status", su::Value(list)}});
}

inline su::Value sonar(const std::vector<std::pair<su::Value, std::string>> & seen)
{
  su::Value::List list;
  for (const auto & [id, waypoint] : seen) {
    list.push_back(su::Value(su::Value::Map{{"id", id}, {"waypoint", su::Value(waypoint)}}));
  }
  return su::Value(su::Value::Map{{"contacts", su::Value(list)}});
}

#endif  // UNIT__HELPERS_HPP_
