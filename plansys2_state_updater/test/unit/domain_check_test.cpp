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

// The checks of the rules against the domain, the problem and the plan

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "helpers.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_state_updater/domain_check.hpp"

using plansys2_msgs::msg::Node;

namespace
{

su::Signature signature(const std::vector<std::string> & types)
{
  su::Signature out;
  for (const auto & type : types) {
    out.emplace_back(type, std::vector<std::string>{});
  }
  return out;
}

bool any(const std::vector<std::string> & lines, const std::string & part)
{
  return std::any_of(
    lines.begin(), lines.end(), [&](const std::string & line) {return contains(line, part);});
}

Node node(uint8_t type, const std::string & name = "", std::vector<uint32_t> children = {})
{
  Node n;
  n.node_type = type;
  n.name = name;
  n.children = children;
  return n;
}

}  // namespace

TEST(domain_check, names_and_parameters)
{
  auto e = engine({
    battery("{mode: correct, stale_after: 5.0, on_stale: '(sensor_stale battery)'}"), leak()});
  su::DomainInfo domain;
  domain.predicates = {{"leak_detected", {}}, {"sensor_stale", signature({"sensor"})}};
  domain.functions = {{"energy", {}}};
  domain.measured_functions = std::set<std::string>{};
  auto [errors, warnings] = su::check_domain(*e, domain);
  EXPECT_TRUE(errors.empty());
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_TRUE(contains(warnings[0], "without min_change"));

  su::DomainInfo other;
  other.predicates = {{"sensor_stale", {}}};
  other.functions = {{"energy", signature({"r"})}};
  other.measured_functions = std::set<std::string>{"energy"};
  errors = su::check_domain(*e, other).first;
  EXPECT_EQ(errors.size(), 4u);
  EXPECT_TRUE(any(errors, "'leak_detected' is not in the domain"));
  EXPECT_TRUE(any(errors, "has 0 parameters"));
  EXPECT_TRUE(any(errors, "has 1 parameters"));
  EXPECT_TRUE(any(errors, "measured_functions"));
}

TEST(domain_check, placeholders_count_as_parameters)
{
  auto e = engine({diagnostics("{stale_after: 1.0, on_stale: '(sensor_stale {name})'}")});
  su::DomainInfo domain;
  domain.predicates = {
    {"sensor_ok", signature({"sensor"})}, {"sensor_stale", signature({"sensor"})}};
  auto [errors, warnings] = su::check_domain(*e, domain);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(warnings.empty());
  domain.predicates["sensor_ok"] = signature({"sensor", "sensor"});
  EXPECT_EQ(su::check_domain(*e, domain).first.size(), 1u);
}

TEST(domain_check, modified_by_reads_the_effects)
{
  // (and (decrease (energy) (* #t (rate))) (not (free r1)) (at_end) (when (dry) (wet)))
  plansys2_msgs::msg::Tree effects;
  effects.nodes = {
    node(Node::AND, "", {1, 5, 7, 8}),
    node(Node::FUNCTION_MODIFIER, "", {2, 3}),
    node(Node::FUNCTION, "energy"),
    node(Node::EXPRESSION, "", {4}),
    node(Node::FUNCTION, "rate"),
    node(Node::NOT, "", {6}),
    node(Node::PREDICATE, "free"),
    node(Node::PREDICATE, "at_end"),
    node(Node::WHEN, "", {9, 10}),
    node(Node::PREDICATE, "dry"),
    node(Node::PREDICATE, "wet"),
  };
  const auto [predicates, functions] = su::modified_by({effects, plansys2_msgs::msg::Tree()});
  EXPECT_EQ(predicates, (std::set<std::string>{"free", "at_end", "wet"}));
  EXPECT_EQ(functions, (std::set<std::string>{"energy"}));
}

TEST(domain_check, rejects_what_the_plan_changes)
{
  auto e = engine({
    battery("{mode: correct, min_change: 2.0}"), leak(),
    "{name: at, topic: /loc, field: wp, target: {enum: '(robot_at r1 {})'}}",
    "{name: depth, topic: /depth, field: z, target: {function: '(depth)'}}"});
  su::DomainInfo domain;
  domain.predicates = {{"leak_detected", {}}, {"robot_at", signature({"robot", "wp"})}};
  domain.functions = {{"energy", {}}, {"depth", {}}};
  domain.modified_predicates = {"robot_at"};
  domain.modified_functions = {"energy", "depth"};
  domain.measured_functions = std::set<std::string>{};
  auto errors = su::check_domain(*e, domain).first;
  EXPECT_EQ(errors.size(), 2u);
  EXPECT_TRUE(any(errors, "predicate 'robot_at' is changed by the actions"));
  EXPECT_TRUE(any(errors, "function 'depth' is changed by the actions"));
  EXPECT_TRUE(any(errors, "mode 'correct'"));
  // Measured, the model does not integrate it: mode write is the one
  domain.measured_functions = std::set<std::string>{"depth"};
  EXPECT_EQ(su::check_domain(*e, domain).first.size(), 1u);
}

TEST(domain_check, types_of_the_objects)
{
  auto e = engine({
    battery("{min_change: 1.0, stale_after: 1.0, on_stale: '(sensor_stale battery)'}"),
    "{name: at, topic: /loc, field: wp, "
    "target: {enum: '(robot_at r1 {})', values: [wp1, kitchen, wp9]}}"});
  su::DomainInfo domain;
  domain.predicates = {
    {"sensor_stale", {{"sensor", {"battery_sensor"}}}},
    {"robot_at", signature({"robot", "waypoint"})}};
  domain.functions = {{"energy", {}}};
  domain.objects = std::map<std::string, std::string>{
    {"battery", "battery_sensor"}, {"r1", "robot"}, {"wp1", "waypoint"}, {"kitchen", "room"}};
  auto [errors, warnings] = su::check_domain(*e, domain);
  EXPECT_EQ(errors, (std::vector<std::string>{
    "rule 'at': predicate 'robot_at': 'kitchen' is a room, the parameter is a waypoint"}));
  EXPECT_EQ(warnings, (std::vector<std::string>{
    "rule 'at': predicate 'robot_at': 'wp9' is not in the problem yet: its writes fail "
    "until it is"}));
  // Unknown objects: nothing to say of the types
  domain.objects.reset();
  std::tie(errors, warnings) = su::check_domain(*e, domain);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(warnings.empty());
}

TEST(domain_check, instance_rules)
{
  auto e = engine({contacts()});
  su::DomainInfo domain;
  domain.predicates = {
    {"contact_at", {{"contact", {"sonar_contact"}}, {"waypoint", {}}}},
    {"detected", signature({"object"})}};
  domain.types = std::set<std::string>{"contact", "waypoint"};
  auto [errors, warnings] = su::check_domain(*e, domain);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(warnings.empty());
  domain.types = std::set<std::string>{"waypoint"};
  domain.predicates["contact_at"] = signature({"robot", "waypoint"});
  domain.modified_predicates = {"detected"};
  errors = su::check_domain(*e, domain).first;
  EXPECT_EQ(errors.size(), 3u);
  EXPECT_TRUE(any(errors, "the type 'contact' is not in the domain"));
  EXPECT_TRUE(any(errors, "its objects are contact, the parameter is a robot"));
  EXPECT_TRUE(any(errors, "'detected' is changed by the actions"));
}

TEST(domain_check, a_type_from_the_messages_is_checked_as_they_come)
{
  auto e = engine({
    "{name: contacts, topic: /sonar, field: 'contacts[*]', "
    "target: {instance: {name: 'contact_{id}', type: '{class}'}, facts: ['(detected {self})']}, "
    "policy: {expire_after: 10.0}}"});
  su::DomainInfo domain;
  domain.predicates = {{"detected", signature({"contact"})}};
  domain.types = std::set<std::string>{"contact"};
  EXPECT_TRUE(su::check_domain(*e, domain).first.empty());
}

TEST(domain_check, object_fluents)
{
  auto e = engine({
    "{name: pos, topic: /loc, field: area, target: {function: '(pos r1)'}, "
    "policy: {stale_after: 5.0, on_stale: {value: lost}}}",
    "{name: where, topic: /loc2, field: area, target: {function: '(pos r2)'}, "
    "policy: {mode: correct, min_change: 1.0}}",
    battery("{stale_after: 5.0, on_stale: {value: empty}}")});
  su::DomainInfo domain;
  domain.functions = {{"pos", signature({"robot"})}, {"energy", {}}};
  domain.object_fluents = {{"pos", "place"}};
  domain.objects = std::map<std::string, std::string>{
    {"r1", "robot"}, {"r2", "robot"}, {"lost", "status"}};
  const auto errors = su::check_domain(*e, domain).first;
  EXPECT_EQ(errors.size(), 4u) << errors.size();
  EXPECT_TRUE(any(errors, "'lost' is a status, the parameter is a place"));
  EXPECT_TRUE(any(errors, "mode 'correct' is for numbers"));
  EXPECT_TRUE(any(errors, "min_change is for numbers"));
  EXPECT_TRUE(any(errors, "'energy' has a number: its on_stale value is a number"));
}
