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

#ifndef PLANSYS2_STATE_UPDATER__INSTANCES_HPP_
#define PLANSYS2_STATE_UPDATER__INSTANCES_HPP_

#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "plansys2_state_updater/paths.hpp"
#include "plansys2_state_updater/rules.hpp"
#include "plansys2_state_updater/value.hpp"

namespace YAML
{
class Node;
}  // namespace YAML

namespace plansys2_state_updater
{

/// An object name with placeholders: 'contact_{id}' takes id from each element
class NameTemplate
{
public:
  explicit NameTemplate(const std::string & text);
  const std::string & text() const {return text_;}
  /// Whether it takes something of the element: else it is always text()
  bool is_template() const;
  std::string ground(const Value & element) const;

private:
  std::string text_;
  std::vector<std::variant<std::string, Path>> parts_;
};

/// An object an instance rule created: its facts, when it was last seen
struct InstanceState
{
  std::string type;  // As it was created: the first message that saw it says it
  double last_seen {0.0};
  bool missing {false};  // Not in the last message, with remove_when_missing: removed
  bool created {false};
  std::optional<double> last_write;
  std::optional<double> retry_at;  // After a creation that failed
  std::set<std::string> facts;
  bool held {false};  // Expired, but the plan or the goal uses it: kept
  double next_use_check {0.0};
};

/// An object a message speaks of: its name, its type and the facts that hold on it
struct Seen
{
  std::string name;
  std::string type;
  std::set<std::string> facts;
};

/// Objects of the problem from the messages: created when seen, removed when not seen for
/// expire_after (or missing, with remove_when_missing) unless the plan or the goal uses them
class InstanceRule : public RuleBase
{
public:
  /// @throws ConfigError with every problem of the rule
  explicit InstanceRule(const YAML::Node & spec);

  std::optional<NameTemplate> name_template;
  std::string type;
  std::optional<NameTemplate> type_template;
  // The types of the domain, if known: an object of another type is not created
  std::optional<std::set<std::string>> types;
  std::vector<Template> fact_templates;
  std::optional<double> expire_after;
  bool remove_when_missing {false};
  size_t max_instances {0};  // 0: no cap
  std::map<std::string, InstanceState> instances;

  std::vector<std::pair<std::string, bool>> owned() const override;
  void start(double) override {}
  double max_age() const override {return expire_after.value_or(0.0);}

  /// Each object the message speaks of, in the order seen; error says the elements left out
  std::vector<Seen> observe(const Value & message);
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__INSTANCES_HPP_
