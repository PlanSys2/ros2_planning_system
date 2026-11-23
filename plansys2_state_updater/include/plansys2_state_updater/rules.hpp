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

#ifndef PLANSYS2_STATE_UPDATER__RULES_HPP_
#define PLANSYS2_STATE_UPDATER__RULES_HPP_

#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "plansys2_state_updater/paths.hpp"
#include "plansys2_state_updater/transforms.hpp"
#include "plansys2_state_updater/value.hpp"

namespace YAML
{
class Node;
}  // namespace YAML

namespace plansys2_state_updater
{

/// A configuration that cannot work: every problem found, one per line
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// The value of a fact: none yet, a predicate that holds or not, a number, a name (an enum)
using FactValue = std::variant<std::monostate, bool, double, std::string>;

/// As the reasons show it: None, True, False, 95, normal
std::string to_text(const FactValue & value);

/// A fact to write: a function with its value, a predicate that holds or not, an instance
struct Change
{
  Change() = default;
  Change(
    std::string rule, std::string kind, std::string fact, FactValue value, std::string reason,
    FactValue previous = {})
  : rule(std::move(rule)), kind(std::move(kind)), fact(std::move(fact)), value(std::move(value)),
    reason(std::move(reason)), previous(std::move(previous)) {}

  std::string rule;
  std::string kind;  // function, predicate or instance
  std::string fact;
  // An instance: its type to create it, or false to remove it
  FactValue value;
  std::string reason;
  // What the fact was before, as far as the rules know (none: unknown, as a first value)
  FactValue previous;
};

/// A fact with placeholders: '(sensor_ok {name})' takes name from each element
class Template
{
public:
  explicit Template(const std::string & fact);

  const std::string & fact() const {return fact_;}
  const std::string & name() const {return name_;}
  /// Each parameter: a name, or a path in the element (empty for '{}', the element itself)
  const std::vector<std::variant<std::string, Path>> & params() const {return params_;}
  bool is_template() const;
  /// How many parameters are this path: '{}' is the empty one
  size_t count(const Path & path) const;

  std::string ground(const Value & element) const;

private:
  std::string fact_;
  std::string name_;
  std::vector<std::variant<std::string, Path>> params_;
};

/// When messages last came, for stale: of a whole rule, or of each fact of a template
struct Watch
{
  double since {0.0};  // Last message, or when it started to wait for one
  bool stale {false};
  std::optional<std::string> stale_fact;
  bool announced {false};  // Whether stale_fact was written: then it is kept in sync
};

/// The policy state of one fact
struct FactState
{
  FactValue value;  // Last value of the fact from the sensor
  FactValue written;
  std::optional<double> last_write;
  FactValue pending_value;
  std::optional<double> pending_since;
  std::shared_ptr<Watch> watch;  // Of its own, in a template
  FactValue before;  // What was written before the last write
  std::optional<double> retry_at;  // After a write that failed
};

/// The facts of a rule, in the order they came: the order of the writes follows it
class FactTable
{
public:
  FactState & get(const std::string & fact);  // Added if it is not there
  FactState * find(const std::string & fact);
  const FactState * find(const std::string & fact) const;
  bool contains(const std::string & fact) const {return index_.count(fact) > 0;}
  size_t size() const {return entries_.size();}

  using Entries = std::vector<std::pair<std::string, std::unique_ptr<FactState>>>;
  Entries::iterator begin() {return entries_.begin();}
  Entries::iterator end() {return entries_.end();}
  Entries::const_iterator begin() const {return entries_.begin();}
  Entries::const_iterator end() const {return entries_.end();}

private:
  Entries entries_;
  std::map<std::string, size_t> index_;
};

/// A fact a message speaks of: its new value, and the element it comes from
struct Observation
{
  std::string fact;
  FactValue value;
  Value element;
};

/// In the order they were seen; a fact seen again keeps its place and takes the new value
class Observed : public std::vector<Observation>
{
public:
  void set(const std::string & fact, FactValue value, Value element = Value());
  bool contains(const std::string & fact) const;
};

/// How the topic of a rule is subscribed. "auto": what all its publishers offer
struct Qos
{
  std::string reliability {"auto"};     // auto, reliable or best_effort
  std::string durability {"volatile"};  // auto, volatile or transient_local
  size_t depth {5};

  bool operator==(const Qos & other) const
  {
    return reliability == other.reliability && durability == other.durability &&
           depth == other.depth;
  }
  bool operator!=(const Qos & other) const {return !(*this == other);}
  /// reliability auto, durability volatile, depth 5
  std::string text() const;
};

/// What a publisher offers
struct Offered
{
  bool reliable {true};
  bool transient_local {false};
};

/// The qos to subscribe with, given the publishers of the topic: "auto" takes the most that all
/// of them offer. Empty: "auto" with no publishers yet, nothing to take it from
std::optional<Qos> resolve(const Qos & qos, const std::vector<Offered> & publishers);

/// What every rule has: what it listens to, its warnings and its last error
class RuleBase
{
public:
  virtual ~RuleBase() = default;

  std::string name;
  std::string topic;
  std::string msg_type;  // Empty: seen in the graph
  std::optional<Qos> qos;  // Empty: not given
  std::string field;
  Path path;
  std::vector<std::string> warnings;
  std::optional<std::string> error;
  std::string spec;  // Its YAML, as written: a reload keeps it if it is the same

  // What it reads: a message each sample_period at most, the others are not even read
  double sample_period {0.0};
  std::optional<double> last_sample;
  // The time of the reading (header.stamp), not of arrival: an old reading does not say the
  // sensor is alive
  std::optional<Path> stamp;
  std::string stamp_field;
  std::optional<double> last_stamp;
  size_t old_messages {0};  // Not taken: older than max_age, or than one already taken

  /// What the rule writes: (fact, whole predicate) for each fact or template
  virtual std::vector<std::pair<std::string, bool>> owned() const = 0;
  virtual void start(double now) = 0;
  /// How old a reading may be, with a stamp
  virtual double max_age() const = 0;

  bool due(double now) const
  {
    return !sample_period || !last_sample || now - *last_sample >= sample_period;
  }
  /// The time of the reading, in seconds: @throws std::exception if the message has none
  double measured(const Value & message) const;

protected:
  /// sample_period and stamp, from the policy
  void parse_input(const YAML::Node & policy, std::vector<std::string> & errors);
  /// The qos of the rule, if given
  void parse_qos(const YAML::Node & spec, std::vector<std::string> & errors);
  /// Each of them against the time that the rule waits for its messages (stale, expire)
  void check_input(
    double waits, const std::string & waits_name, std::vector<std::string> & errors) const;
};

/// Facts kept in sync with a field of a topic, and how they are written
class Rule : public RuleBase
{
public:
  /// @throws ConfigError with every problem of the rule
  explicit Rule(const YAML::Node & spec);

  std::string kind;  // function or predicate (a set and an enum are predicates)
  std::string fact;
  std::string fact_name;
  std::optional<Template> fact_template;
  bool is_set {false};
  bool is_enum {false};
  std::vector<std::string> enum_values;
  std::unique_ptr<Transform> transform;
  // A function whose value is an object, (= (pos r1) kitchen): what the domain says of it
  bool object_fluent {false};

  std::string mode {"write"};
  double min_change {0.0};
  double debounce {0.0};
  double min_period {0.0};
  double stale_after {0.0};
  std::optional<std::string> on_stale;
  std::optional<Template> stale_template;
  bool has_stale_value {false};
  Value stale_value;

  // Each fact is watched for stale on its own in a template; else, the whole rule
  bool per_fact_watch {false};
  std::shared_ptr<Watch> watch;
  FactTable facts;

  std::vector<std::pair<std::string, bool>> owned() const override;
  void start(double now) override;
  double max_age() const override {return stale_after;}

  /// Each fact the message speaks of, with its new value and its element
  Observed observe(const Value & message);
  /// The facts of one value: of the fact (a function or predicate), or of an enum
  Observed observe_value(const Value & value, const std::optional<std::string> & fact = {});
  /// The facts of the on_stale value for the facts a watch covers
  Observed stale_values(const std::shared_ptr<Watch> & watch);

private:
  std::vector<std::string> parse_on_stale(const YAML::Node & on_stale);
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__RULES_HPP_
