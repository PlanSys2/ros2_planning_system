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

#include "plansys2_state_updater/rules.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace plansys2_state_updater
{

namespace
{

const std::regex kPlaceholder(R"(^\{([^{}]*)\}$)");
const std::vector<std::string> kPolicyKeys = {
  "mode", "min_change", "debounce", "min_period", "stale_after", "on_stale", "sample_period",
  "stamp"};

std::string
join(const std::vector<std::string> & items, const std::string & separator)
{
  std::string out;
  for (const auto & item : items) {
    out += (out.empty() ? "" : separator) + item;
  }
  return out;
}

std::string
listed(const std::vector<std::string> & items)
{
  std::string out = "[";
  for (const auto & item : items) {
    out += (out.size() > 1 ? ", '" : "'") + item + "'";
  }
  return out + "]";
}

std::string
scalar(const YAML::Node & node, const std::string & otherwise = "")
{
  return node && node.IsScalar() ? node.Scalar() : otherwise;
}

double
number(const YAML::Node & node, const std::string & what, std::vector<std::string> & errors)
{
  if (!node || node.IsNull()) {
    return 0.0;
  }
  try {
    return from_yaml(node).as_number();
  } catch (const std::invalid_argument &) {
    errors.push_back(what + " is a number of seconds or a quantity, not '" + scalar(node) + "'");
    return 0.0;
  }
}

}  // namespace

std::string
to_text(const FactValue & value)
{
  if (std::holds_alternative<std::monostate>(value)) {
    return "None";
  }
  if (std::holds_alternative<bool>(value)) {
    return std::get<bool>(value) ? "True" : "False";
  }
  if (std::holds_alternative<double>(value)) {
    return format_g(std::get<double>(value));
  }
  return std::get<std::string>(value);
}

void
RuleBase::parse_input(const YAML::Node & policy, std::vector<std::string> & errors)
{
  if (!policy || !policy.IsMap()) {
    return;
  }
  if (policy["sample_period"]) {
    try {
      sample_period = from_yaml(policy["sample_period"]).as_number();
    } catch (const std::invalid_argument &) {
      sample_period = -1.0;
    }
    if (!(sample_period >= 0.0)) {
      errors.push_back("sample_period is a number of seconds, 0 or more");
      sample_period = 0.0;
    }
  }
  if (policy["stamp"]) {
    stamp_field = scalar(policy["stamp"]);
    try {
      stamp = parse_path(stamp_field);
      if (stamp_field.empty() || has_wildcard(*stamp)) {
        throw std::invalid_argument(stamp_field);
      }
    } catch (const std::invalid_argument &) {
      stamp.reset();
      errors.push_back(
        "stamp is the field of the message with the time of the reading, like header.stamp, "
        "without [*]");
    }
  }
}

std::string
Qos::text() const
{
  return "reliability " + reliability + ", durability " + durability + ", depth " +
         std::to_string(depth);
}

std::optional<Qos>
resolve(const Qos & qos, const std::vector<Offered> & publishers)
{
  const bool adapts = qos.reliability == "auto" || qos.durability == "auto";
  if (adapts && publishers.empty()) {
    return std::nullopt;
  }
  Qos out = qos;
  if (qos.reliability == "auto") {
    const bool all = std::all_of(
      publishers.begin(), publishers.end(), [](const Offered & p) {return p.reliable;});
    out.reliability = all ? "reliable" : "best_effort";
  }
  if (qos.durability == "auto") {
    const bool all = std::all_of(
      publishers.begin(), publishers.end(), [](const Offered & p) {return p.transient_local;});
    out.durability = all ? "transient_local" : "volatile";
  }
  return out;
}

void
RuleBase::parse_qos(const YAML::Node & spec, std::vector<std::string> & errors)
{
  const YAML::Node given = spec["qos"];
  if (!given || given.IsNull()) {
    return;
  }
  if (!given.IsMap()) {
    errors.push_back(
      "qos is {reliability: auto|reliable|best_effort, "
      "durability: auto|volatile|transient_local, depth: messages}");
    return;
  }
  const std::vector<std::string> keys = {"reliability", "durability", "depth"};
  std::vector<std::string> unknown;
  for (const auto & item : given) {
    const std::string key = item.first.Scalar();
    if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
      unknown.push_back(key);
    }
  }
  if (!unknown.empty()) {
    errors.push_back("unknown qos keys " + listed(unknown) + ", known: " + listed(keys));
  }
  Qos result;
  auto one_of = [&](const std::string & key, std::string & value,
    const std::vector<std::string> & values) {
      if (!given[key]) {
        return;
      }
      const std::string text = scalar(given[key]);
      if (std::find(values.begin(), values.end(), text) == values.end()) {
        errors.push_back("qos " + key + " is one of " + listed(values) + ", not '" + text + "'");
      } else {
        value = text;
      }
    };
  one_of("reliability", result.reliability, {"auto", "reliable", "best_effort"});
  one_of("durability", result.durability, {"auto", "volatile", "transient_local"});
  if (given["depth"]) {
    Value depth;
    try {
      depth = from_yaml(given["depth"]);
    } catch (const std::invalid_argument &) {
    }
    if (!depth.is_int() || depth.as_number() < 1) {
      errors.push_back("qos depth is a number of messages, 1 or more");
    } else {
      result.depth = static_cast<size_t>(depth.as_number());
    }
  }
  qos = result;
}

void
RuleBase::check_input(
  double waits, const std::string & waits_name, std::vector<std::string> & errors) const
{
  if (stamp && !waits) {
    errors.push_back("stamp needs " + waits_name + ": how old a reading may be");
  }
  if (sample_period && waits && waits <= sample_period) {
    errors.push_back(
      waits_name + " must be longer than sample_period: a message is read each sample_period "
      "at most");
  }
}

double
RuleBase::measured(const Value & message) const
{
  const Value time = get_field(message, *stamp);
  if (time.is_map() && time.as_map().count("sec") && time.as_map().count("nanosec")) {
    return time.as_map().at("sec").as_number() + time.as_map().at("nanosec").as_number() * 1e-9;
  }
  if (time.is_number() && !time.is_bool()) {
    return time.as_number();
  }
  throw std::invalid_argument(
    "it is " + time.kind() + ", not a time: {sec, nanosec} or a number of seconds");
}

Template::Template(const std::string & fact)
: fact_(fact)
{
  const auto [name, params] = parse_fact(fact);
  name_ = name;
  for (const auto & param : params) {
    std::smatch match;
    if (!std::regex_match(param, match, kPlaceholder)) {
      params_.emplace_back(param);
      continue;
    }
    Path path = parse_path(match[1].str());
    if (has_wildcard(path)) {
      throw std::invalid_argument("'" + param + "' in '" + fact + "' cannot have [*]");
    }
    params_.emplace_back(path);
  }
}

bool
Template::is_template() const
{
  for (const auto & param : params_) {
    if (std::holds_alternative<Path>(param)) {
      return true;
    }
  }
  return false;
}

size_t
Template::count(const Path & path) const
{
  size_t n = 0;
  for (const auto & param : params_) {
    if (std::holds_alternative<Path>(param) && std::get<Path>(param) == path) {
      n++;
    }
  }
  return n;
}

std::string
Template::ground(const Value & element) const
{
  std::string fact = "(" + name_;
  for (const auto & param : params_) {
    fact += " " + (std::holds_alternative<std::string>(param) ?
      std::get<std::string>(param) :
      as_name(get_field(element, std::get<Path>(param)), fact_));
  }
  return fact + ")";
}

FactState &
FactTable::get(const std::string & fact)
{
  auto it = index_.find(fact);
  if (it != index_.end()) {
    return *entries_[it->second].second;
  }
  index_[fact] = entries_.size();
  entries_.emplace_back(fact, std::make_unique<FactState>());
  return *entries_.back().second;
}

FactState *
FactTable::find(const std::string & fact)
{
  auto it = index_.find(fact);
  return it == index_.end() ? nullptr : entries_[it->second].second.get();
}

const FactState *
FactTable::find(const std::string & fact) const
{
  auto it = index_.find(fact);
  return it == index_.end() ? nullptr : entries_[it->second].second.get();
}

void
Observed::set(const std::string & fact, FactValue value, Value element)
{
  for (auto & observation : *this) {
    if (observation.fact == fact) {
      observation.value = std::move(value);
      observation.element = std::move(element);
      return;
    }
  }
  push_back(Observation{fact, std::move(value), std::move(element)});
}

bool
Observed::contains(const std::string & fact) const
{
  for (const auto & observation : *this) {
    if (observation.fact == fact) {
      return true;
    }
  }
  return false;
}

Rule::Rule(const YAML::Node & spec)
{
  std::vector<std::string> errors;
  name = scalar(spec["name"]);
  if (name.empty()) {
    errors.push_back("a rule has no name");
  }
  topic = scalar(spec["topic"]);
  if (topic.empty()) {
    errors.push_back("no topic");
  }
  msg_type = scalar(spec["type"]);  // Optional: else, discovered in the graph
  parse_qos(spec, errors);
  // Optional: without it, the whole message (for a plugin)
  field = scalar(spec["field"]);
  try {
    path = parse_path(field);
  } catch (const std::invalid_argument & e) {
    errors.push_back(e.what());
  }
  try {
    transform = std::make_unique<Transform>(spec["transform"]);
  } catch (const std::invalid_argument & e) {
    errors.push_back(e.what());
  }

  const YAML::Node target = spec["target"];
  std::vector<std::string> kinds;
  for (const auto & k : {"function", "predicate", "set", "enum"}) {
    if (target && target.IsMap() && target[k]) {
      kinds.push_back(k);
    }
  }
  if (kinds.size() != 1) {
    errors.push_back(
      "target must be {function: \"(f ...)\"}, {predicate: \"(p ...)\"}, "
      "{set: \"(p {})\"} or {enum: \"(p {})\"}");
  } else {
    fact = scalar(target[kinds[0]]);
    is_set = kinds[0] == "set";
    is_enum = kinds[0] == "enum";
    kind = kinds[0] == "function" ? "function" : "predicate";
    try {
      fact_template.emplace(fact);
      fact_name = fact_template->name();
    } catch (const std::invalid_argument & e) {
      errors.push_back(e.what());
    }
    if (is_enum && target["values"]) {
      std::vector<std::string> bad;
      for (const auto & value : target["values"]) {
        enum_values.push_back(from_yaml(value).text());
        if (!is_name(enum_values.back())) {
          bad.push_back(enum_values.back());
        }
      }
      if (!bad.empty()) {
        errors.push_back("enum values that are not PDDL names: " + listed(bad));
      }
    }
    std::vector<std::string> unknown;
    for (const auto & item : target) {
      const std::string key = item.first.Scalar();
      if (key != kinds[0] && !(is_enum && key == "values")) {
        unknown.push_back(key);
      }
    }
    if (!unknown.empty()) {
      errors.push_back("unknown target keys " + listed(unknown));
    }
  }

  const YAML::Node policy = spec["policy"];
  if (policy && policy.IsMap()) {
    std::vector<std::string> unknown;
    for (const auto & item : policy) {
      const std::string key = item.first.Scalar();
      if (std::find(kPolicyKeys.begin(), kPolicyKeys.end(), key) == kPolicyKeys.end()) {
        unknown.push_back(key);
      }
    }
    if (!unknown.empty()) {
      errors.push_back(
        "unknown policy keys " + listed(unknown) + ", known: " + listed(kPolicyKeys));
    }
  }
  const YAML::Node no_policy;
  const YAML::Node & p = policy && policy.IsMap() ? policy : no_policy;
  mode = p.IsMap() ? scalar(p["mode"], "write") : "write";
  if (mode != "write" && mode != "correct") {
    errors.push_back("mode must be 'write' or 'correct'");
  }
  min_change = p.IsMap() ? number(p["min_change"], "min_change", errors) : 0.0;
  debounce = p.IsMap() ? number(p["debounce"], "debounce", errors) : 0.0;
  min_period = p.IsMap() ? number(p["min_period"], "min_period", errors) : 0.0;
  stale_after = p.IsMap() ? number(p["stale_after"], "stale_after", errors) : 0.0;
  for (const auto & e : parse_on_stale(p.IsMap() ? p["on_stale"] : YAML::Node())) {
    errors.push_back(e);
  }
  parse_input(p, errors);
  check_input(stale_after, "stale_after", errors);

  const bool many = has_wildcard(path);
  const bool templated = fact_template && fact_template->is_template();
  const bool stale_templated = stale_template && stale_template->is_template();
  const bool transformed = transform && !transform->kind().empty();
  if (is_set) {
    if (!templated) {
      errors.push_back("the fact of a set needs a placeholder, like (reachable {})");
    }
    if (transformed) {
      errors.push_back("a set takes no transform: its elements are its facts");
    }
    if (stale_templated) {
      errors.push_back("on_stale of a set is one fact for the whole list, no placeholder");
    }
    if (has_stale_value) {
      errors.push_back("on_stale of a set takes no value: the list is the value");
    }
  } else if (is_enum) {
    if (fact_template && fact_template->count(Path{}) != 1) {
      errors.push_back("the fact of an enum has one '{}', the value: (mode_is {})");
    } else if (fact_template) {
      for (const auto & param : fact_template->params()) {
        if (std::holds_alternative<Path>(param) && !std::get<Path>(param).empty()) {
          errors.push_back("the fact of an enum has no placeholder but '{}'");
          break;
        }
      }
    }
    if (many) {
      errors.push_back("'" + field + "' gives many values: an enum has one at a time");
    }
    if (transform && transform->gives_bool().has_value()) {
      errors.push_back(
        "transform '" + transform->kind() + "' gives a " +
        (*transform->gives_bool() ? "boolean" : "number") +
        ": an enum takes a name, from map, range, a plugin or the field");
    }
    if (stale_templated) {
      errors.push_back("on_stale of an enum is one fact, no placeholder");
    }
    if (has_stale_value && !enum_values.empty() &&
      std::find(enum_values.begin(), enum_values.end(), stale_value.text()) == enum_values.end())
    {
      errors.push_back("the on_stale value '" + stale_value.text() + "' is not in the values");
    }
  } else if (many && !templated) {
    errors.push_back(
      "'" + field + "' gives many values: the fact needs a placeholder, "
      "like (sensor_ok {name}), or the target a set");
  }
  if (stale_templated && !templated) {
    errors.push_back("on_stale has a placeholder and the fact has none");
  }

  if (kind == "function") {
    if (transform && transform->gives_bool().value_or(false)) {
      errors.push_back(
        "transform '" + transform->kind() + "' gives a boolean: the target must be a predicate");
    }
    if (debounce) {
      errors.push_back("debounce is for predicates: for functions use min_change");
    }
    // A name, for a function whose value is an object: the domain says which one it is
    if (has_stale_value && !(stale_value.is_number() && !stale_value.is_bool()) &&
      !(stale_value.is_string() && is_name(stale_value.as_string())))
    {
      errors.push_back(
        "the on_stale value of a function is a number (an object, for an object fluent)");
    }
    if (mode == "correct" && !min_change) {
      warnings.push_back(
        "rule '" + name + "': mode 'correct' without min_change writes every reading, as the "
        "model moves the value between them");
    }
  } else if (kind == "predicate") {
    if (mode == "correct") {
      errors.push_back("mode 'correct' is for functions, which the plan also changes");
    }
    if (min_change) {
      errors.push_back("min_change is for functions");
    }
    if (has_stale_value && !is_enum && !stale_value.is_bool()) {
      errors.push_back("the on_stale value of a predicate is true or false");
    }
  }

  if (!errors.empty()) {
    std::vector<std::string> lines;
    for (const auto & e : errors) {
      lines.push_back("rule '" + name + "': " + e);
    }
    throw ConfigError(join(lines, "\n"));
  }
  per_fact_watch = templated && !is_set && !is_enum;
}

std::vector<std::string>
Rule::parse_on_stale(const YAML::Node & node)
{
  if (!node || node.IsNull() || (node.IsScalar() && node.Scalar() == "keep")) {
    return {};
  }
  const std::vector<std::string> shape = {
    "on_stale must be 'keep', a predicate like '(sensor_stale b)', {value: ...}, "
    "or {predicate: ..., value: ...}"};
  // A default YAML::Node is a defined null: flags say what there is
  std::vector<std::string> errors;
  bool has_predicate = false, has_value = false;
  YAML::Node predicate, value;
  if (node.IsScalar()) {
    has_predicate = true;
    predicate = node;
  } else if (node.IsMap() && node.size() > 0) {
    for (const auto & item : node) {
      const std::string key = item.first.Scalar();
      if (key == "predicate") {
        has_predicate = true;
        predicate = item.second;
      } else if (key == "value") {
        has_value = true;
        value = item.second;
      } else {
        return shape;
      }
    }
  }
  if (!has_predicate && !has_value) {
    return shape;
  }
  if (has_predicate) {
    on_stale = scalar(predicate);
    try {
      stale_template.emplace(*on_stale);
    } catch (const std::invalid_argument &) {
      errors.push_back(
        "on_stale predicate '" + *on_stale + "' is not like '(sensor_stale b)'");
    }
  }
  if (has_value) {
    has_stale_value = true;
    stale_value = from_yaml(value);
  }
  if (!stale_after) {
    errors.push_back("on_stale needs stale_after");
  }
  return errors;
}

std::vector<std::pair<std::string, bool>>
Rule::owned() const
{
  std::vector<std::pair<std::string, bool>> owned = {{fact, fact_template->is_template()}};
  if (stale_template) {
    owned.emplace_back(*on_stale, stale_template->is_template());
  }
  return owned;
}

void
Rule::start(double now)
{
  if (!per_fact_watch) {
    watch = std::make_shared<Watch>();
    watch->since = now;
    if (stale_template) {
      watch->stale_fact = on_stale;
    }
  }
  if (!fact_template->is_template()) {
    facts.get(fact);
  }
  for (const auto & value : enum_values) {
    facts.get(fact_template->ground(Value(value)));
  }
}

Observed
Rule::observe(const Value & message)
{
  Observed observed;
  std::vector<std::string> errors;
  std::vector<std::pair<Value, Value>> items = get_items(message, path);
  if (is_set && !has_wildcard(path)) {
    // 'waypoints', a list: its elements, as 'waypoints[*]'
    const Value list = items[0].second;
    if (!list.is_list()) {
      throw std::out_of_range("'" + field + "' is " + list.kind() + ", not a list");
    }
    items.clear();
    for (const auto & raw : list.as_list()) {
      items.emplace_back(raw, raw);
    }
  }
  for (const auto & [element, raw] : items) {
    try {
      if (is_set) {
        // The element of the list itself is the placeholder '{}'
        observed.set(fact_template->ground(raw), true, raw);
      } else if (is_enum) {
        return observe_value(transform->apply(raw));
      } else {
        const std::string grounded = fact_template->ground(element);
        const FactState * state = facts.find(grounded);
        std::optional<bool> active;
        if (state && std::holds_alternative<bool>(state->value)) {
          active = std::get<bool>(state->value);
        }
        const Value value = transform->apply(raw, active);
        // With its element: the stale predicate of a template is made of it
        observed.set(grounded, observe_value(value, grounded)[0].value, element);
      }
    } catch (const std::exception & e) {
      errors.push_back("'" + field + "': " + e.what());
    }
  }
  if (is_set) {
    for (const auto & [grounded, state] : facts) {
      const bool held = std::holds_alternative<bool>(state->value) &&
        std::get<bool>(state->value);
      const bool written = std::holds_alternative<bool>(state->written) &&
        std::get<bool>(state->written);
      if (!observed.contains(grounded) && (held || written)) {
        observed.set(grounded, false);
      }
    }
  }
  error = errors.empty() ? std::nullopt : std::optional<std::string>(join(errors, "; "));
  return observed;
}

Observed
Rule::observe_value(const Value & value, const std::optional<std::string> & grounded)
{
  Observed observed;
  if (!is_enum) {
    if (kind == "function" && object_fluent) {
      observed.set(grounded.value_or(fact), as_name(value, fact));
    } else if (kind == "function") {
      const double number = value.as_number();
      if (!std::isfinite(number)) {
        throw std::invalid_argument(value.text() + " is not a finite number");
      }
      observed.set(grounded.value_or(fact), number);
    } else {
      observed.set(grounded.value_or(fact), value.truthy());
    }
    return observed;
  }
  const std::string name = as_name(value, fact);
  if (!enum_values.empty() &&
    std::find(enum_values.begin(), enum_values.end(), name) == enum_values.end())
  {
    throw std::invalid_argument("'" + name + "' is not one of the values " + listed(enum_values));
  }
  const std::string holds = fact_template->ground(Value(name));
  // The others first, so that two never hold at once; the ones never written, too, so that
  // the one of the problem file goes
  for (const auto & [other, state] : facts) {
    const bool written_false = std::holds_alternative<bool>(state->written) &&
      !std::get<bool>(state->written);
    if (other != holds && !written_false) {
      observed.set(other, false);
    }
  }
  observed.set(holds, true, Value(name));
  error = std::nullopt;
  return observed;
}

Observed
Rule::stale_values(const std::shared_ptr<Watch> & of)
{
  if (per_fact_watch) {
    Observed observed;
    for (const auto & [grounded, state] : facts) {
      if (state->watch == of) {
        observed.set(grounded, observe_value(stale_value, grounded)[0].value);
      }
    }
    return observed;
  }
  return observe_value(stale_value);
}

}  // namespace plansys2_state_updater
