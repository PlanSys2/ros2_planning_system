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

#include "plansys2_state_updater/engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace plansys2_state_updater
{

namespace
{

std::string
seconds(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f", value);
  return buffer;
}

bool
is_true(const FactValue & value)
{
  return std::holds_alternative<bool>(value) && std::get<bool>(value);
}

Value
as_value(const FactValue & value)
{
  if (std::holds_alternative<bool>(value)) {return Value(std::get<bool>(value));}
  if (std::holds_alternative<double>(value)) {return Value(std::get<double>(value));}
  if (std::holds_alternative<std::string>(value)) {return Value(std::get<std::string>(value));}
  return Value();
}

Value
as_value(const std::optional<double> & value)
{
  return value ? Value(*value) : Value();
}

}  // namespace

Engine::Engine(
  std::vector<std::unique_ptr<RuleBase>> rules, CurrentValue current_value, InUse in_use)
: rules_(std::move(rules)), current_value_(std::move(current_value)), in_use_(std::move(in_use))
{
  std::vector<std::string> errors;
  std::set<std::string> names;
  // A fact is owned by one rule; a template owns its whole predicate
  std::map<std::pair<std::string, std::vector<std::string>>, std::string> exact;
  std::map<std::string, std::string> whole;
  // One subscription per topic: the rules that give its qos must agree
  std::map<std::string, std::pair<Qos, std::string>> qos_of;
  for (const auto & rule : rules_) {
    if (!names.insert(rule->name).second) {
      errors.push_back("two rules are called '" + rule->name + "'");
    }
    if (rule->qos) {
      auto [it, first] = qos_of.emplace(rule->topic, std::make_pair(*rule->qos, rule->name));
      if (!first && it->second.first != *rule->qos) {
        errors.push_back(
          "rules '" + it->second.second + "' and '" + rule->name + "' read '" + rule->topic +
          "' with another qos (" + it->second.first.text() + "; " + rule->qos->text() +
          "): a topic is subscribed once");
      }
    }
    for (const auto & [fact, is_template] : rule->owned()) {
      const auto [name, params] = parse_fact(fact);
      std::set<std::string> others;
      if (is_template) {
        for (const auto & [key, owner] : exact) {
          if (key.first == name && owner != rule->name) {
            others.insert(owner);
          }
        }
        if (whole.count(name) && whole[name] != rule->name) {
          others.insert(whole[name]);
        }
        whole[name] = rule->name;
      } else {
        auto it = exact.find({name, params});
        if (it != exact.end() && it->second != rule->name) {
          others.insert(it->second);
        }
        if (whole.count(name) && whole[name] != rule->name) {
          others.insert(whole[name]);
        }
        exact[{name, params}] = rule->name;
      }
      for (const auto & other : others) {
        errors.push_back(
          "'" + fact + "' is written by rules '" + other + "' and '" + rule->name +
          "': each fact has one owner (a template, its whole predicate)");
      }
    }
  }
  if (!errors.empty()) {
    std::string text;
    for (const auto & e : errors) {
      text += (text.empty() ? "" : "\n") + e;
    }
    throw ConfigError(text);
  }
}

std::unique_ptr<Engine>
Engine::from_config(const YAML::Node & config, CurrentValue current_value, InUse in_use)
{
  YAML::Node root = config;
  if (root && root.IsMap() && root["state_updater"]) {
    root = root["state_updater"];
  }
  const YAML::Node specs = root && root.IsMap() ? root["rules"] : YAML::Node();
  if (!specs || !specs.IsSequence() || specs.size() == 0) {
    throw ConfigError("no rules");
  }
  std::vector<std::unique_ptr<RuleBase>> rules;
  std::string errors;
  for (const auto & spec : specs) {
    try {
      if (!spec.IsMap()) {
        throw ConfigError("a rule is a map: {name: ..., topic: ..., target: ...}");
      }
      const YAML::Node target = spec["target"];
      if (target && target.IsMap() && target["instance"]) {
        rules.push_back(std::make_unique<InstanceRule>(spec));
      } else {
        rules.push_back(std::make_unique<Rule>(spec));
      }
      rules.back()->spec = YAML::Dump(spec);
    } catch (const ConfigError & e) {
      errors += (errors.empty() ? "" : "\n") + std::string(e.what());
    }
  }
  if (!errors.empty()) {
    throw ConfigError(errors);
  }
  return std::make_unique<Engine>(std::move(rules), current_value, in_use);
}

std::map<std::string, std::string>
Engine::topics() const
{
  std::map<std::string, std::string> topics;
  for (const auto & rule : rules_) {
    auto & type = topics[rule->topic];
    if (type.empty()) {
      type = rule->msg_type;
    }
  }
  return topics;
}

std::map<std::string, Qos>
Engine::qos() const
{
  std::map<std::string, Qos> qos;
  for (const auto & rule : rules_) {
    if (rule->qos) {
      qos[rule->topic] = *rule->qos;
    } else {
      qos.emplace(rule->topic, Qos());
    }
  }
  return qos;
}

void
Engine::start(double now)
{
  last_now_ = now;
  for (auto & rule : rules_) {
    rule->start(now);
  }
}

void
Engine::follow_clock(double now)
{
  if (last_now_ && now < *last_now_) {
    const double back = now - *last_now_;
    auto shift = [back](std::optional<double> & time) {
        if (time) {
          *time += back;
        }
      };
    for (auto & base : rules_) {
      shift(base->last_sample);
      base->last_stamp.reset();
      if (auto * instances = dynamic_cast<InstanceRule *>(base.get())) {
        for (auto & [name, state] : instances->instances) {
          state.last_seen += back;
          state.next_use_check += back;
          shift(state.last_write);
          shift(state.retry_at);
        }
        continue;
      }
      auto & rule = dynamic_cast<Rule &>(*base);
      for (auto & watch : watches(rule)) {
        watch->since += back;
      }
      for (auto & [fact, state] : rule.facts) {
        shift(state->last_write);
        shift(state->pending_since);
        shift(state->retry_at);
      }
    }
  }
  last_now_ = now;
}

std::set<std::string>
Engine::take_over(Engine & previous, double now)
{
  previous.follow_clock(now);  // What it kept, on the clock as it is now
  last_now_ = now;
  std::set<std::string> kept;
  for (auto & rule : rules_) {
    auto same = std::find_if(
      previous.rules_.begin(), previous.rules_.end(), [&](const auto & old) {
        return old && old->name == rule->name && old->spec == rule->spec;
      });
    if (same != previous.rules_.end()) {
      rule = std::move(*same);
      kept.insert(rule->name);
    } else {
      rule->start(now);
    }
  }
  return kept;
}

std::vector<std::string>
Engine::set_disabled(const std::vector<std::string> & names, double now)
{
  const std::set<std::string> wanted(names.begin(), names.end());
  std::set<std::string> known;
  for (auto & rule : rules_) {
    known.insert(rule->name);
    if (disabled_.count(rule->name) && !wanted.count(rule->name)) {
      if (auto * facts = dynamic_cast<Rule *>(rule.get())) {
        for (auto & watch : watches(*facts)) {
          watch->since = now;
        }
      } else if (auto * instances = dynamic_cast<InstanceRule *>(rule.get())) {
        for (auto & [name, state] : instances->instances) {
          state.last_seen = now;
          state.missing = false;
        }
      }
    }
  }
  disabled_.clear();
  std::vector<std::string> unknown;
  for (const auto & name : wanted) {
    if (known.count(name)) {
      disabled_.insert(name);
    } else {
      unknown.push_back(name);
    }
  }
  return unknown;
}

std::vector<std::shared_ptr<Watch>>
Engine::watches(const Rule & rule) const
{
  if (rule.watch) {
    return {rule.watch};
  }
  std::vector<std::shared_ptr<Watch>> watches;
  for (const auto & [fact, state] : rule.facts) {
    if (state->watch) {
      watches.push_back(state->watch);
    }
  }
  return watches;
}

std::vector<Change>
Engine::on_message(const std::string & topic, const Value & message, double now)
{
  return receive(topic, [&message]() {return message;}, now);
}

std::vector<Change>
Engine::receive(const std::string & topic, const Reader & read, double now)
{
  follow_clock(now);
  std::vector<RuleBase *> due;
  for (auto & base : rules_) {
    if (base->topic != topic || !enabled(*base)) {
      continue;
    }
    if (base->due(now)) {
      due.push_back(base.get());
      continue;
    }
    // Not read, but its topic speaks: the whole rule is not stale (with a stamp, when the
    // reading was made is not known without reading it)
    auto * rule = dynamic_cast<Rule *>(base.get());
    if (rule && rule->watch && !rule->watch->stale && !rule->stamp) {
      rule->watch->since = now;
    }
  }
  if (due.empty()) {
    return {};
  }
  const Value message = read();
  std::vector<Change> changes;
  for (auto * base : due) {
    base->last_sample = now;
    auto taken = take_message(*base, message, now);
    changes.insert(changes.end(), taken.begin(), taken.end());
  }
  return changes;
}

std::vector<Change>
Engine::take_message(RuleBase & base, const Value & message, double now)
{
  double since = now;
  if (base.stamp) {
    try {
      since = std::min(now, base.measured(message));
    } catch (const std::exception & e) {
      base.error = "'" + base.stamp_field + "': " + e.what();
      return {};
    }
    // Too old, or older than one taken: what it says is not what there is now
    if (now - since >= base.max_age()) {
      base.old_messages++;
      base.error = "a reading " + seconds(now - since) + " s old, by its stamp, is not taken (" +
        (dynamic_cast<InstanceRule *>(&base) ? "expire_after " : "stale_after ") +
        format_g(base.max_age()) + "): is '" + base.stamp_field + "' filled?";
      return {};
    }
    if (base.last_stamp && since < *base.last_stamp) {
      base.old_messages++;
      return {};
    }
    base.last_stamp = since;
  }

  if (auto * instances = dynamic_cast<InstanceRule *>(&base)) {
    std::vector<Seen> observed;
    try {
      observed = instances->observe(message);
    } catch (const std::exception & e) {
      instances->error = "'" + instances->field + "': " + e.what();
      return {};
    }
    // An element left out (a bad name) may be one that is there: none is missing then.
    // The missing ones go first: they leave room for the new ones (max_instances)
    std::vector<Change> changes;
    if (instances->remove_when_missing && !instances->error) {
      std::set<std::string> names;
      for (const auto & object : observed) {
        names.insert(object.name);
      }
      for (auto & [name, state] : instances->instances) {
        state.missing = !names.count(name);
        if (!state.missing) {
          state.last_seen = std::max(state.last_seen, since);
        }
      }
      changes = expire(*instances, now);
    }
    auto seen_changes = seen(*instances, observed, now, since);
    changes.insert(changes.end(), seen_changes.begin(), seen_changes.end());
    return changes;
  }

  auto & rule = dynamic_cast<Rule &>(base);
  std::vector<Change> changes;
  if (rule.watch) {
    changes = fresh(rule, *rule.watch, since);
  }
  Observed observed;
  try {
    observed = rule.observe(message);
  } catch (const std::exception & e) {
    rule.error = "'" + rule.field + "': " + e.what();
    return changes;
  }
  auto taken = take(rule, observed, now, since);
  changes.insert(changes.end(), taken.begin(), taken.end());
  return changes;
}

std::vector<Change>
Engine::take(Rule & rule, const Observed & observed, double now, std::optional<double> since)
{
  std::vector<Change> changes;
  for (const auto & observation : observed) {
    FactState & state = rule.facts.get(observation.fact);
    if (rule.per_fact_watch && since) {
      if (!state.watch) {
        state.watch = std::make_shared<Watch>();
        state.watch->since = *since;
        if (rule.stale_template) {
          try {
            state.watch->stale_fact = rule.stale_template->ground(observation.element);
          } catch (const std::exception & e) {
            rule.error = "'" + rule.field + "': " + e.what();
          }
        }
      }
      auto fresh_changes = fresh(rule, *state.watch, *since);
      changes.insert(changes.end(), fresh_changes.begin(), fresh_changes.end());
    }
    state.value = observation.value;
    auto decided = decide(rule, observation.fact, state, now);
    changes.insert(changes.end(), decided.begin(), decided.end());
  }
  return changes;
}

std::vector<Change>
Engine::fresh(const Rule & rule, Watch & watch, double since)
{
  watch.since = std::max(watch.since, since);
  if (!watch.stale) {
    return {};
  }
  watch.stale = false;
  if (!watch.stale_fact) {
    return {};
  }
  watch.announced = true;
  return {Change{rule.name, "predicate", *watch.stale_fact, false,
      rule.name + ": messages again", true}};
}

std::vector<Change>
Engine::decide(Rule & rule, const std::string & fact, FactState & state, double now)
{
  if (state.retry_at && now < *state.retry_at) {
    if (!state.pending_since) {
      state.pending_since = now;
    }
    return {};
  }
  state.retry_at.reset();
  const FactValue value = state.value;
  const std::string label = fact == rule.fact ? rule.name : rule.name + " " + fact;
  std::string reason;
  FactValue previous = state.written;
  // An object fluent changes as a predicate does: when its object is another
  const bool number_valued = rule.kind == "function" && std::holds_alternative<double>(value);
  if (number_valued) {
    const double number = std::get<double>(value);
    std::optional<double> reference;
    if (std::holds_alternative<double>(state.written)) {
      reference = std::get<double>(state.written);
    }
    if (rule.mode == "correct" && current_value_) {
      reference = current_value_(fact);
    }
    if (reference) {
      const double deviation = std::abs(number - *reference);
      if (deviation < rule.min_change || deviation == 0.0) {
        state.pending_since.reset();
        return {};
      }
      reason = label + ": " + format_g(*reference) + " → " + format_g(number) + ", deviation " +
        format_g(deviation) + " ≥ " + format_g(rule.min_change);
      previous = *reference;  // In mode correct, what the problem has: the model's
    } else {
      reason = label + ": first value " + format_g(number);
    }
  } else {
    // As written; or one that was going to hold, and goes before it was ever written
    const bool cancelled = std::holds_alternative<std::monostate>(state.written) &&
      value == FactValue(false) && state.pending_value == FactValue(true);
    if (value == state.written || cancelled) {
      state.pending_value = std::monostate();
      state.pending_since.reset();
      return {};
    }
    if (state.pending_value != value || !state.pending_since) {
      state.pending_value = value;
      state.pending_since = now;
    }
    if (now - *state.pending_since < rule.debounce) {
      return {};
    }
    reason = label + ": " + to_text(state.written) + " → " + to_text(value);
    if (rule.debounce) {
      reason += ", held " + seconds(now - *state.pending_since) + " s ≥ " +
        format_g(rule.debounce);
    }
  }

  if (state.last_write && now - *state.last_write < rule.min_period) {
    // Written in a later tick, if it still has to
    if (!state.pending_since) {
      state.pending_since = now;
    }
    return {};
  }

  state.pending_value = std::monostate();
  state.pending_since.reset();
  state.before = state.written;
  state.written = value;
  state.last_write = now;
  return {Change{rule.name, rule.kind, fact, value, reason, previous}};
}

std::vector<Change>
Engine::seen(InstanceRule & rule, const std::vector<Seen> & observed, double now, double since)
{
  std::vector<Change> changes;
  std::vector<std::string> notes;
  std::string refused;
  for (const auto & [name, type, facts] : observed) {
    auto it = rule.instances.find(name);
    if (it == rule.instances.end()) {
      if (rule.max_instances && rule.instances.size() >= rule.max_instances) {
        refused += (refused.empty() ? "" : ", ") + name;
        continue;
      }
      it = rule.instances.emplace(name, InstanceState{}).first;
      it->second.type = type;
    }
    InstanceState & state = it->second;
    if (state.type != type) {
      if (state.created) {
        notes.push_back(name + " is a " + state.type + ", the message says " + type + ": kept");
      } else {
        state.type = type;
      }
    }
    state.last_seen = std::max(state.last_seen, since);
    state.missing = false;
    state.held = false;
    if (!state.created) {
      if (state.retry_at && now < *state.retry_at) {
        continue;
      }
      state.created = true;
      state.last_write = now;
      state.retry_at.reset();
      changes.push_back(
        Change{rule.name, "instance", name, state.type,
          rule.name + ": " + name + " - " + state.type + " seen"});
    }
    std::vector<std::string> gone, added;
    std::set_difference(
      state.facts.begin(), state.facts.end(), facts.begin(), facts.end(),
      std::back_inserter(gone));
    std::set_difference(
      facts.begin(), facts.end(), state.facts.begin(), state.facts.end(),
      std::back_inserter(added));
    for (const auto & fact : gone) {
      state.facts.erase(fact);
      changes.push_back(Change{rule.name, "predicate", fact, false,
          rule.name + ": " + fact + " no longer", true});
    }
    for (const auto & fact : added) {
      state.facts.insert(fact);
      changes.push_back(Change{rule.name, "predicate", fact, true, rule.name + ": " + fact,
          false});
    }
    state.last_write = now;
  }
  if (!refused.empty()) {
    notes.push_back(
      "max_instances " + std::to_string(rule.max_instances) + ": not created " + refused);
  }
  for (const auto & note : notes) {
    rule.error = (rule.error ? *rule.error + "; " : "") + note;
  }
  return changes;
}

std::vector<Change>
Engine::expire(InstanceRule & rule, double now)
{
  std::vector<Change> changes;
  for (auto it = rule.instances.begin(); it != rule.instances.end(); ) {
    auto & [name, state] = *it;
    const double unseen = now - state.last_seen;
    const bool expired = rule.expire_after && unseen >= *rule.expire_after;
    if (!expired && !state.missing) {
      ++it;
      continue;
    }
    if (in_use_) {
      if (now < state.next_use_check) {
        ++it;
        continue;
      }
      state.next_use_check = now + kInUsePeriod;
      if (in_use_(name)) {
        state.held = true;
        ++it;
        continue;
      }
    }
    if (state.created) {
      for (const auto & fact : state.facts) {
        changes.push_back(Change{rule.name, "predicate", fact, false,
            rule.name + ": " + fact + ", as " + name + " goes", true});
      }
      std::string reason = expired ?
        rule.name + ": " + name + " not seen for " + seconds(unseen) + " s (expire_after " +
        format_g(*rule.expire_after) + ")" :
        rule.name + ": " + name + " not in the last message (remove_when_missing)";
      if (state.held) {
        reason += ", and no longer used by the plan or the goal";
      }
      changes.push_back(Change{rule.name, "instance", name, false, reason});
    }
    it = rule.instances.erase(it);
  }
  return changes;
}

std::vector<Change>
Engine::on_timer(double now)
{
  follow_clock(now);
  std::vector<Change> changes;
  for (auto & base : rules_) {
    if (!enabled(*base)) {
      continue;
    }
    if (auto * instances = dynamic_cast<InstanceRule *>(base.get())) {
      auto expired = expire(*instances, now);
      changes.insert(changes.end(), expired.begin(), expired.end());
      continue;
    }
    auto & rule = dynamic_cast<Rule &>(*base);
    for (auto & [fact, state] : rule.facts) {
      if (state->pending_since) {
        auto decided = decide(rule, fact, *state, now);
        changes.insert(changes.end(), decided.begin(), decided.end());
      }
    }
    for (auto & watch : watches(rule)) {
      if (!rule.stale_after || watch->stale || now - watch->since < rule.stale_after) {
        continue;
      }
      watch->stale = true;
      const std::string reason = rule.name + ": no message for " + seconds(now - watch->since) +
        " s (stale_after " + format_g(rule.stale_after) + ")";
      if (watch->stale_fact) {
        watch->announced = true;
        changes.push_back(Change{rule.name, "predicate", *watch->stale_fact, true, reason, false});
      }
      if (rule.has_stale_value) {
        for (auto change : take(rule, rule.stale_values(watch), now, std::nullopt)) {
          change.reason += ", stale: its on_stale value (" + reason + ")";
          changes.push_back(change);
        }
      }
    }
  }
  return changes;
}

void
Engine::write_failed(const Change & change, double now, double retry_after)
{
  for (auto & base : rules_) {
    if (base->name != change.rule) {
      continue;
    }
    if (auto * instances = dynamic_cast<InstanceRule *>(base.get())) {
      if (change.kind == "instance" && std::holds_alternative<std::string>(change.value)) {
        auto it = instances->instances.find(change.fact);
        if (it != instances->instances.end()) {
          // Created again later, and its facts with it
          it->second.created = false;
          it->second.retry_at = now + retry_after;
          it->second.facts.clear();
        }
      } else if (change.kind == "predicate" && is_true(change.value)) {
        for (auto & [name, state] : instances->instances) {
          state.facts.erase(change.fact);  // Written again with the next message
        }
      }
      continue;
    }
    auto & rule = dynamic_cast<Rule &>(*base);
    FactState * state = rule.facts.find(change.fact);
    if (state && state->written == change.value) {
      state->written = state->before;
      state->retry_at = now + retry_after;
      state->pending_since = now;
    }
  }
}

std::vector<Change>
Engine::resync(const ProblemState & problem, double now, double min_age)
{
  follow_clock(now);
  const auto & predicates = problem.predicates;
  const auto & functions = problem.functions;
  const auto & instances = problem.instances;
  std::vector<Change> changes;
  for (auto & base : rules_) {
    if (!enabled(*base)) {
      continue;
    }
    if (auto * instance_rule = dynamic_cast<InstanceRule *>(base.get())) {
      for (auto & [name, state] : instance_rule->instances) {
        if (!state.created || !state.last_write || now - *state.last_write < min_age) {
          continue;
        }
        const bool gone = instances && !instances->count(name);
        std::vector<Change> again;
        if (gone) {
          again.push_back(Change{instance_rule->name, "instance", name, state.type,
              instance_rule->name + ": resync, " + name + " left the problem"});
        }
        for (const auto & fact : state.facts) {
          if (gone || !predicates.count(canonical(fact))) {
            again.push_back(Change{instance_rule->name, "predicate", fact, true,
                instance_rule->name + ": resync, " + fact + " left the problem", false});
          }
        }
        if (!again.empty()) {
          state.last_write = now;
          changes.insert(changes.end(), again.begin(), again.end());
        }
      }
      continue;
    }
    auto & rule = dynamic_cast<Rule &>(*base);
    for (auto & [fact, state] : rule.facts) {
      if (std::holds_alternative<std::monostate>(state->written) || state->pending_since ||
        !state->last_write || now - *state->last_write < min_age)
      {
        continue;
      }
      const std::string key = canonical(fact);
      std::string reason;
      FactValue found;  // What the problem has
      if (rule.kind == "predicate") {
        const bool holds = predicates.count(key) > 0;
        if (holds == is_true(state->written)) {
          continue;
        }
        found = holds;
        reason = rule.name + ": resync, " + fact +
          (is_true(state->written) ? " left the problem" : " is in the problem again");
      } else if (std::holds_alternative<std::string>(state->written)) {
        // An object fluent
        auto it = problem.objects.find(key);
        const auto & written = std::get<std::string>(state->written);
        if (it != problem.objects.end() && it->second == written) {
          continue;
        }
        if (it != problem.objects.end()) {
          found = it->second;
        }
        reason = rule.name + ": resync, " + fact +
          (it == problem.objects.end() ? " left the problem" : " is " + it->second + " in it");
      } else {
        auto it = functions.find(key);
        const double written = std::get<double>(state->written);
        if (it != functions.end() &&
          (rule.mode == "correct" ||
          std::abs(it->second - written) <= std::max(1e-6, 1e-6 * std::abs(written))))
        {
          continue;
        }
        if (it != functions.end()) {
          found = it->second;
        }
        reason = rule.name + ": resync, " + fact +
          (it == functions.end() ? " left the problem" : " is " + format_g(it->second) + " in it");
        if (rule.mode == "correct" && std::holds_alternative<double>(state->value)) {
          state->written = state->value;  // The last reading, not the last write
        }
      }
      state->last_write = now;
      changes.push_back(Change{rule.name, rule.kind, fact, state->written, reason, found});
    }
    for (auto & watch : watches(rule)) {
      if (watch->stale_fact && watch->announced &&
        (predicates.count(canonical(*watch->stale_fact)) > 0) != watch->stale)
      {
        changes.push_back(Change{rule.name, "predicate", *watch->stale_fact, watch->stale,
            rule.name + ": resync, " + *watch->stale_fact, !watch->stale});
      }
    }
  }
  return changes;
}

Value
Engine::status() const
{
  Value::List rules;
  for (const auto & base : rules_) {
    Value::Map entry;
    entry["rule"] = Value(base->name);
    entry["enabled"] = Value(enabled(*base));
    entry["error"] = base->error ? Value(*base->error) : Value();
    if (base->stamp) {
      entry["old_messages"] = Value(base->old_messages);
    }
    if (const auto * instance_rule = dynamic_cast<const InstanceRule *>(base.get())) {
      Value::Map objects;
      for (const auto & [name, state] : instance_rule->instances) {
        Value::List facts;
        for (const auto & fact : state.facts) {
          facts.push_back(Value(fact));
        }
        objects[name] = Value(Value::Map{
            {"type", Value(state.type)}, {"last_seen", Value(state.last_seen)},
            {"created", Value(state.created)}, {"facts", Value(facts)},
            {"held", Value(state.held)}});
      }
      entry["instances"] = Value(objects);
    } else {
      const auto & rule = dynamic_cast<const Rule &>(*base);
      bool stale = false;
      for (const auto & watch : watches(rule)) {
        stale = stale || watch->stale;
      }
      entry["stale"] = Value(stale);
      Value::Map facts;
      for (const auto & [fact, state] : rule.facts) {
        facts[fact] = Value(Value::Map{
            {"value", as_value(state->value)}, {"written", as_value(state->written)},
            {"last_write", as_value(state->last_write)},
            {"stale", state->watch ? Value(state->watch->stale) : Value()}});
      }
      entry["facts"] = Value(facts);
    }
    rules.push_back(Value(entry));
  }
  return Value(rules);
}

}  // namespace plansys2_state_updater
