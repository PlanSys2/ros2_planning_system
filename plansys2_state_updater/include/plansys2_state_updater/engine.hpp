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

#ifndef PLANSYS2_STATE_UPDATER__ENGINE_HPP_
#define PLANSYS2_STATE_UPDATER__ENGINE_HPP_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_state_updater/instances.hpp"
#include "plansys2_state_updater/rules.hpp"
#include "plansys2_state_updater/value.hpp"

namespace YAML
{
class Node;
}  // namespace YAML

namespace plansys2_state_updater
{

/// The problem as it is now, for resync
struct ProblemState
{
  std::set<std::string> predicates;
  std::map<std::string, double> functions;
  std::map<std::string, std::string> objects;  // Object fluents: (pos r1) -> kitchen
  std::optional<std::set<std::string>> instances;  // nullopt: unknown
};

/// The rules, fed with messages and the time, give the changes to write; without ROS: a message
/// is a value (a map), the time a number of seconds
class Engine
{
public:
  /// The value of a function in the problem, for mode 'correct'
  using CurrentValue = std::function<std::optional<double>(const std::string &)>;
  /// Whether the plan in course or the goal uses an object, which is then kept
  using InUse = std::function<bool(const std::string &)>;
  /// A message, read when some rule takes it: not at all if all wait for their sample_period
  using Reader = std::function<Value()>;

  /// Seconds between the checks of whether an object to remove is still in use
  static constexpr double kInUsePeriod = 1.0;

  /// @throws ConfigError if two rules are called the same, or write the same facts
  explicit Engine(
    std::vector<std::unique_ptr<RuleBase>> rules, CurrentValue current_value = {},
    InUse in_use = {});

  /// From {state_updater: {rules: [...]}} or {rules: [...]}.
  /// @throws ConfigError with every problem of every rule.
  static std::unique_ptr<Engine> from_config(
    const YAML::Node & config, CurrentValue current_value = {}, InUse in_use = {});

  const std::vector<std::unique_ptr<RuleBase>> & rules() const {return rules_;}

  /// The topics to subscribe, with their type if a rule gives it (empty: seen in the graph)
  std::map<std::string, std::string> topics() const;
  /// The qos of each topic: what its rules give, else the defaults
  std::map<std::string, Qos> qos() const;

  void start(double now);

  /// A reload: each rule with the same name and YAML as one of previous takes its state from it
  /// (moved out of previous); the others start. Gives the names of the ones kept.
  std::set<std::string> take_over(Engine & previous, double now);

  /// Disable these rules and enable the others; gives the names of no rule. A disabled rule
  /// writes nothing; enabled again, it counts its stale and expire times from now.
  std::vector<std::string> set_disabled(const std::vector<std::string> & names, double now);
  const std::set<std::string> & disabled() const {return disabled_;}

  std::vector<Change> on_message(const std::string & topic, const Value & message, double now);
  /// A message read only if some rule takes it now (sample_period).
  /// @throws what read throws, before any rule takes it.
  std::vector<Change> receive(const std::string & topic, const Reader & read, double now);

  /// What time alone changes: values waiting for debounce or min_period, stale, expired objects
  std::vector<Change> on_timer(double now);

  /// Take back a change that could not be written: tried again after retry_after
  void write_failed(const Change & change, double now, double retry_after = 5.0);

  /// The writes again of what the problem lost or someone else changed; a fact written less
  /// than min_age ago is left, as the problem may not show it yet.
  std::vector<Change> resync(const ProblemState & problem, double now, double min_age = 1.0);

  /// For ~/status: each rule, enabled, error, stale, its facts or objects
  Value status() const;

private:
  bool enabled(const RuleBase & rule) const {return !disabled_.count(rule.name);}

  /// A clock that goes back (a sim or bag restarting): every time kept goes back as much, so
  /// none waits for the old time; the last stamp, of another clock, is forgotten.
  void follow_clock(double now);

  std::vector<Change> take_message(RuleBase & rule, const Value & message, double now);
  /// fresh: when the values were read (now, or their stamp); none, values of no message (stale)
  std::vector<Change> take(
    Rule & rule, const Observed & observed, double now, std::optional<double> fresh);
  std::vector<Change> fresh(const Rule & rule, Watch & watch, double since);
  std::vector<Change> decide(Rule & rule, const std::string & fact, FactState & state, double now);
  std::vector<Change> seen(
    InstanceRule & rule, const std::vector<Seen> & observed, double now, double since);
  std::vector<Change> expire(InstanceRule & rule, double now);
  std::vector<std::shared_ptr<Watch>> watches(const Rule & rule) const;

  std::vector<std::unique_ptr<RuleBase>> rules_;
  CurrentValue current_value_;
  InUse in_use_;
  std::set<std::string> disabled_;
  std::optional<double> last_now_;
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__ENGINE_HPP_
