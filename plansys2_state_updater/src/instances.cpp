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

#include "plansys2_state_updater/instances.hpp"

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

const std::regex kPart(R"(\{([^{}]*)\})");

std::string
scalar(const YAML::Node & node)
{
  return node && node.IsScalar() ? node.Scalar() : "";
}

}  // namespace

NameTemplate::NameTemplate(const std::string & text)
: text_(text)
{
  if (text.empty()) {
    throw std::invalid_argument("an instance needs a name, like 'contact_{id}'");
  }
  size_t last = 0;
  for (std::sregex_iterator it(text.begin(), text.end(), kPart), end; it != end; ++it) {
    parts_.emplace_back(text.substr(last, it->position() - last));
    Path path = parse_path((*it)[1].str());
    if (has_wildcard(path)) {
      throw std::invalid_argument("'" + it->str() + "' in '" + text + "' cannot have [*]");
    }
    parts_.emplace_back(path);
    last = it->position() + it->length();
  }
  parts_.emplace_back(text.substr(last));
  if (parts_.size() == 1 && !is_name(text)) {
    throw std::invalid_argument("'" + text + "' is not a valid PDDL name");
  }
}

bool
NameTemplate::is_template() const
{
  for (const auto & part : parts_) {
    if (std::holds_alternative<Path>(part)) {
      return true;
    }
  }
  return false;
}

std::string
NameTemplate::ground(const Value & element) const
{
  if (parts_.size() == 1) {
    return text_;  // Nothing of the element: a name, checked when the rule was made
  }
  std::string name;
  for (const auto & part : parts_) {
    name += std::holds_alternative<std::string>(part) ?
      std::get<std::string>(part) : get_field(element, std::get<Path>(part)).text();
  }
  return as_name(Value(name), text_);
}

InstanceRule::InstanceRule(const YAML::Node & spec)
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
  msg_type = scalar(spec["type"]);
  parse_qos(spec, errors);
  field = scalar(spec["field"]);
  try {
    path = parse_path(field);
  } catch (const std::invalid_argument & e) {
    errors.push_back(e.what());
  }
  if (spec["transform"] && !spec["transform"].IsNull()) {
    errors.push_back("an instance rule takes no transform: its elements are its objects");
  }

  // A node that is not there throws on [], in yaml-cpp: an empty map stands for it
  const YAML::Node target = spec["target"] && spec["target"].IsMap() ?
    spec["target"] : YAML::Node(YAML::NodeType::Map);
  const YAML::Node instance = target["instance"];
  if (!instance || !instance.IsMap() || instance.size() != 2 || !instance["name"] ||
    !instance["type"])
  {
    errors.push_back("instance must be {name: \"contact_{id}\", type: contact}");
  } else {
    try {
      name_template.emplace(scalar(instance["name"]));
    } catch (const std::invalid_argument & e) {
      errors.push_back(e.what());
    }
    type = scalar(instance["type"]);
    try {
      type_template.emplace(type);
    } catch (const std::invalid_argument & e) {
      errors.push_back(
        type.find('{') == std::string::npos ?
        "the type '" + type + "' is not a PDDL name" : std::string(e.what()));
    }
  }
  if (target["facts"]) {
    for (const auto & item : target["facts"]) {
      const std::string fact = scalar(item);
      try {
        Template fact_template(fact);
        if (fact_template.count(Path{PathStep{PathStep::FIELD, "self", 0}}) == 0) {
          errors.push_back("the fact '" + fact + "' of an instance has its object, {self}");
        }
        fact_templates.push_back(fact_template);
      } catch (const std::invalid_argument & e) {
        errors.push_back(e.what());
      }
    }
  }
  std::vector<std::string> unknown;
  for (const auto & item : target) {
    const std::string key = item.first.Scalar();
    if (key != "instance" && key != "facts") {
      unknown.push_back("'" + key + "'");
    }
  }
  if (!unknown.empty()) {
    std::string keys;
    for (const auto & key : unknown) {
      keys += (keys.empty() ? "" : ", ") + key;
    }
    errors.push_back("unknown target keys [" + keys + "]");
  }

  const YAML::Node policy = spec["policy"];
  if (policy && policy.IsMap()) {
    const std::set<std::string> known = {
      "expire_after", "remove_when_missing", "max_instances", "sample_period", "stamp"};
    std::string unknown_keys;
    for (const auto & item : policy) {
      const std::string key = item.first.Scalar();
      if (!known.count(key)) {
        unknown_keys += (unknown_keys.empty() ? "'" : ", '") + key + "'";
      }
    }
    if (!unknown_keys.empty()) {
      errors.push_back(
        "unknown policy keys [" + unknown_keys + "]: an instance rule takes expire_after, "
        "remove_when_missing, max_instances, sample_period and stamp");
    }
    if (policy["expire_after"]) {
      try {
        expire_after = from_yaml(policy["expire_after"]).as_number();
        if (*expire_after <= 0.0) {
          errors.push_back("expire_after is a number of seconds, more than 0");
        }
      } catch (const std::invalid_argument &) {
        errors.push_back("expire_after is a number of seconds, more than 0");
      }
    }
    if (policy["remove_when_missing"]) {
      const Value value = from_yaml(policy["remove_when_missing"]);
      if (!value.is_bool()) {
        errors.push_back("remove_when_missing is true or false");
      } else {
        remove_when_missing = value.as_bool();
      }
    }
    if (policy["max_instances"]) {
      const Value value = from_yaml(policy["max_instances"]);
      if (!value.is_int() || value.as_number() < 1) {
        errors.push_back("max_instances is a number of objects, 1 or more");
      } else {
        max_instances = static_cast<size_t>(value.as_number());
      }
    }
    parse_input(policy, errors);
  }
  check_input(expire_after.value_or(0.0), "expire_after", errors);
  if (!expire_after && !remove_when_missing) {
    warnings.push_back(
      "rule '" + name + "': without expire_after or remove_when_missing, its objects are never "
      "removed");
  }
  if (!errors.empty()) {
    std::string lines;
    for (const auto & e : errors) {
      lines += (lines.empty() ? "" : "\n") + ("rule '" + name + "': " + e);
    }
    throw ConfigError(lines);
  }
}

std::vector<std::pair<std::string, bool>>
InstanceRule::owned() const
{
  std::vector<std::pair<std::string, bool>> owned;
  for (const auto & fact_template : fact_templates) {
    owned.emplace_back(fact_template.fact(), true);
  }
  return owned;
}

std::vector<Seen>
InstanceRule::observe(const Value & message)
{
  std::vector<Seen> observed;
  std::string errors;
  for (const auto & [element, raw] : get_items(message, path)) {
    (void)raw;
    try {
      const std::string object = name_template->ground(element);
      const std::string object_type = type_template->ground(element);
      if (types && !types->count(object_type)) {
        throw std::invalid_argument(
          "the type '" + object_type + "' of " + object + " is not in the domain");
      }
      Value::Map scope = element.is_map() ? element.as_map() : Value::Map{};
      scope["self"] = Value(object);
      std::set<std::string> facts;
      for (const auto & fact_template : fact_templates) {
        facts.insert(fact_template.ground(Value(scope)));
      }
      observed.push_back(Seen{object, object_type, facts});
    } catch (const std::exception & e) {
      errors += (errors.empty() ? "" : "; ") + ("'" + field + "': " + e.what());
    }
  }
  error = errors.empty() ? std::nullopt : std::optional<std::string>(errors);
  return observed;
}

}  // namespace plansys2_state_updater
