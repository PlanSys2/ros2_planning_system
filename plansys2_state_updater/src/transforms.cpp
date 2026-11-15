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

#include "plansys2_state_updater/transforms.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "yaml-cpp/yaml.h"

namespace plansys2_state_updater
{

namespace
{

const std::set<std::string> kKinds = {
  "linear", "compare", "map", "range", "hysteresis", "plugin"};
const std::set<std::string> kOps = {"<", "<=", ">", ">=", "==", "!="};

bool
has(const Value & args, const std::string & key)
{
  return args.is_map() && args.as_map().count(key) > 0;
}

const Value &
arg(const Value & args, const std::string & key)
{
  return args.as_map().at(key);
}

// One loader for all the plugins: it must live as long as them
pluginlib::ClassLoader<TransformPlugin> &
plugin_loader()
{
  static pluginlib::ClassLoader<TransformPlugin> loader(
    "plansys2_state_updater", "plansys2_state_updater::TransformPlugin");
  return loader;
}

bool
compare(const Value & value, const std::string & op, const Value & other)
{
  // Numbers by their value; strings by their text; anything else, only == and !=
  if (value.is_number() && other.is_number()) {
    const double a = value.as_number(), b = other.as_number();
    if (op == "<") {return a < b;}
    if (op == "<=") {return a <= b;}
    if (op == ">") {return a > b;}
    if (op == ">=") {return a >= b;}
    return op == "==" ? a == b : a != b;
  }
  if (value.is_string() && other.is_string()) {
    const auto & a = value.as_string(), & b = other.as_string();
    if (op == "<") {return a < b;}
    if (op == "<=") {return a <= b;}
    if (op == ">") {return a > b;}
    if (op == ">=") {return a >= b;}
    return op == "==" ? a == b : a != b;
  }
  if (op == "==" || op == "!=") {
    return (value == other) == (op == "==");
  }
  throw std::invalid_argument(
          "cannot compare " + value.kind() + " " + op + " " + other.kind());
}

}  // namespace

Transform::Transform(const YAML::Node & spec)
{
  if (!spec || spec.IsNull()) {
    return;
  }
  if (!spec.IsMap() || spec.size() != 1) {
    throw std::invalid_argument(
            "transform must be one of linear, compare, map, range, hysteresis, plugin, "
            "as {kind: {...}}");
  }
  const auto entry = *spec.begin();
  kind_ = entry.first.Scalar();
  if (!kKinds.count(kind_)) {
    throw std::invalid_argument(
            "unknown transform '" + kind_ +
            "', one of linear, compare, map, range, hysteresis, plugin");
  }
  args_ = entry.second.IsNull() ? Value(Value::Map{}) : from_yaml(entry.second);
  if (!args_.is_map()) {
    throw std::invalid_argument("the arguments of " + kind_ + " are a map: {name: value}");
  }
  if (has(args_, "field")) {
    field_ = parse_path(arg(args_, "field").text());
    if (has_wildcard(*field_)) {
      throw std::invalid_argument("the field of a transform cannot have [*]");
    }
  }

  if (kind_ == "linear") {
    if (has(args_, "round") && !arg(args_, "round").is_int()) {
      throw std::invalid_argument("round is a number of decimals");
    }
  } else if (kind_ == "compare") {
    if (!has(args_, "op") || !kOps.count(arg(args_, "op").text())) {
      throw std::invalid_argument("compare needs op, one of < <= > >= == !=");
    }
    if (!has(args_, "value")) {
      throw std::invalid_argument("compare needs value");
    }
  } else if (kind_ == "map") {
    const YAML::Node values = entry.second["values"];
    if (!values || !values.IsMap()) {
      throw std::invalid_argument("map needs values: {sensor value: fact value}");
    }
    for (const auto & item : values) {
      map_values_.emplace_back(from_yaml(item.first), from_yaml(item.second));
    }
  } else if (kind_ == "range") {
    const bool shaped = has(args_, "limits") && has(args_, "values") &&
      arg(args_, "limits").is_list() && arg(args_, "values").is_list() &&
      arg(args_, "values").as_list().size() == arg(args_, "limits").as_list().size() + 1;
    if (!shaped) {
      throw std::invalid_argument(
              "range needs limits: [a, b...] and one value more: "
              "[below a, from a to b..., from the last up]");
    }
    const Value limits_value = arg(args_, "limits");
    const auto & limits = limits_value.as_list();
    for (size_t i = 0; i < limits.size(); i++) {
      if (!limits[i].is_number() || limits[i].is_bool() ||
        (i > 0 && limits[i - 1].as_number() >= limits[i].as_number()))
      {
        throw std::invalid_argument(
                "the limits of a range are numbers, in increasing order");
      }
    }
  } else if (kind_ == "hysteresis") {
    if (has(args_, "on_below") && has(args_, "off_above")) {
      if (!(arg(args_, "on_below").as_number() < arg(args_, "off_above").as_number())) {
        throw std::invalid_argument("hysteresis needs on_below < off_above");
      }
    } else if (has(args_, "on_above") && has(args_, "off_below")) {
      if (!(arg(args_, "off_below").as_number() < arg(args_, "on_above").as_number())) {
        throw std::invalid_argument("hysteresis needs off_below < on_above");
      }
    } else {
      throw std::invalid_argument(
              "hysteresis needs on_below + off_above, or on_above + off_below");
    }
  } else if (kind_ == "plugin") {
    if (!has(args_, "function") || !arg(args_, "function").is_string()) {
      throw std::invalid_argument("a plugin needs function: 'package/Class'");
    }
    if (has(args_, "args") && !arg(args_, "args").is_map()) {
      throw std::invalid_argument("the args of a plugin are a map: {name: value}");
    }
    const std::string name = arg(args_, "function").as_string();
    try {
      plugin_ = plugin_loader().createSharedInstance(name);
    } catch (const pluginlib::PluginlibException & e) {
      throw std::invalid_argument("cannot load the plugin '" + name + "': " + e.what());
    }
  }
}

std::optional<bool>
Transform::gives_bool() const
{
  if (kind_ == "compare" || kind_ == "hysteresis") {
    return true;
  }
  if (kind_ == "linear") {
    return false;
  }
  return std::nullopt;
}

Value
Transform::apply(const Value & input, std::optional<bool> active) const
{
  const Value value = field_ ? get_field(input, *field_) : input;
  if (kind_.empty()) {
    return value;
  }
  if (kind_ == "linear") {
    double result = value.as_number() *
      (has(args_, "scale") ? arg(args_, "scale").as_number() : 1.0) +
      (has(args_, "offset") ? arg(args_, "offset").as_number() : 0.0);
    if (has(args_, "round")) {
      const double scale = std::pow(10.0, arg(args_, "round").as_number());
      result = std::round(result * scale) / scale;
    }
    return Value(result);
  }
  if (kind_ == "compare") {
    return Value(compare(value, arg(args_, "op").text(), arg(args_, "value")));
  }
  if (kind_ == "map") {
    // The YAML keys, as values: 2 matches 2 and 2.0, "normal" matches normal
    for (const auto & [key, mapped] : map_values_) {
      if (key == value || key.text() == value.text()) {
        return mapped;
      }
    }
    if (has(args_, "default")) {
      return arg(args_, "default");
    }
    throw std::invalid_argument("no value for '" + value.text() + "' in map, and no default");
  }
  if (kind_ == "range") {
    const double number = value.as_number();
    const Value limits_value = arg(args_, "limits"), values_value = arg(args_, "values");
    const auto & limits = limits_value.as_list();
    const auto & values = values_value.as_list();
    for (size_t i = 0; i < limits.size(); i++) {
      if (number < limits[i].as_number()) {
        return values[i];
      }
    }
    return values.back();
  }
  if (kind_ == "plugin") {
    try {
      return plugin_->apply(value, has(args_, "args") ? arg(args_, "args") : Value(Value::Map{}));
    } catch (const std::exception & e) {
      throw std::invalid_argument(
              "plugin '" + arg(args_, "function").text() + "': " + e.what());
    }
  }
  // hysteresis
  const double number = value.as_number();
  if (has(args_, "on_below")) {
    if (number < arg(args_, "on_below").as_number()) {
      return Value(true);
    }
    return Value(number > arg(args_, "off_above").as_number() || !active ? false : *active);
  }
  if (number > arg(args_, "on_above").as_number()) {
    return Value(true);
  }
  return Value(number < arg(args_, "off_below").as_number() || !active ? false : *active);
}

}  // namespace plansys2_state_updater
