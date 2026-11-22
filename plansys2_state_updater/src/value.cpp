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

#include "plansys2_state_updater/value.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace plansys2_state_updater
{

Value::Value(bool value)
: data_(value) {}

Value::Value(const char * value)
: data_(std::string(value)) {}

Value::Value(std::string value)
: data_(std::move(value)) {}

Value::Value(List value)
: data_(std::make_shared<const List>(std::move(value))) {}

Value::Value(Map value)
: data_(std::make_shared<const Map>(std::move(value))) {}

bool
Value::as_bool() const
{
  if (!is_bool()) {
    throw std::invalid_argument(text() + " is not true or false");
  }
  return std::get<bool>(data_);
}

double
Value::as_number() const
{
  if (is_bool()) {
    return std::get<bool>(data_) ? 1.0 : 0.0;
  }
  if (is_int()) {
    return static_cast<double>(std::get<int64_t>(data_));
  }
  if (is_double()) {
    return std::get<double>(data_);
  }
  if (is_string()) {
    // As Python's float(): a number written as text
    const auto & s = std::get<std::string>(data_);
    double number = 0.0;
    auto [end, error] = std::from_chars(s.data(), s.data() + s.size(), number);
    if (error == std::errc() && end == s.data() + s.size() && !s.empty()) {
      return number;
    }
  }
  throw std::invalid_argument("'" + text() + "' is not a number");
}

const std::string &
Value::as_string() const
{
  if (!is_string()) {
    throw std::invalid_argument(text() + " is not a string");
  }
  return std::get<std::string>(data_);
}

const Value::List &
Value::as_list() const
{
  if (!is_list()) {
    throw std::invalid_argument(text() + " is not a list");
  }
  return *std::get<std::shared_ptr<const List>>(data_);
}

const Value::Map &
Value::as_map() const
{
  if (!is_map()) {
    throw std::invalid_argument(text() + " is not a map");
  }
  return *std::get<std::shared_ptr<const Map>>(data_);
}

bool
Value::truthy() const
{
  if (is_null()) {
    return false;
  }
  if (is_number()) {
    return as_number() != 0.0;
  }
  if (is_string()) {
    return !as_string().empty();
  }
  if (is_list()) {
    return !as_list().empty();
  }
  return !as_map().empty();
}

namespace
{

std::string
real_text(double value)
{
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value > 0 ? "inf" : "-inf";
  }
  // The shortest that reads back the same, as Python's repr: 0.1, not 0.1000000000000000055
  char buffer[64];
  auto [end, error] = std::to_chars(buffer, buffer + sizeof(buffer), value);
  std::string text(buffer, end);
  if (text.find_first_of(".e") == std::string::npos) {
    text += ".0";
  }
  return text;
}

std::string
quoted(const std::string & text)
{
  std::string out = "\"";
  for (char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      case '\r': out += "\\r"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char escaped[8];
          std::snprintf(escaped, sizeof(escaped), "\\u%04x", c);
          out += escaped;
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

}  // namespace

std::string
Value::text() const
{
  if (is_null()) {
    return "None";
  }
  if (is_bool()) {
    return std::get<bool>(data_) ? "true" : "false";
  }
  if (is_int()) {
    return std::to_string(std::get<int64_t>(data_));
  }
  if (is_double()) {
    return real_text(std::get<double>(data_));
  }
  if (is_string()) {
    return std::get<std::string>(data_);
  }
  return json();
}

std::string
Value::json() const
{
  if (is_null()) {
    return "null";
  }
  if (is_double()) {
    const double value = std::get<double>(data_);
    return std::isfinite(value) ? real_text(value) : "null";
  }
  if (is_string()) {
    return quoted(as_string());
  }
  if (is_list()) {
    std::string out = "[";
    for (const auto & item : as_list()) {
      out += (out.size() > 1 ? ", " : "") + item.json();
    }
    return out + "]";
  }
  if (is_map()) {
    std::string out = "{";
    for (const auto & [key, item] : as_map()) {
      out += (out.size() > 1 ? ", " : "") + quoted(key) + ": " + item.json();
    }
    return out + "}";
  }
  return text();
}

std::string
Value::kind() const
{
  if (is_null()) {return "nothing";}
  if (is_bool()) {return "true or false";}
  if (is_number()) {return "a number";}
  if (is_string()) {return "a string";}
  if (is_list()) {return "a list";}
  return "a map";
}

bool
Value::operator==(const Value & other) const
{
  // Numbers by their value, whatever their kind, as Python: 2 == 2.0, true == 1
  if (is_number() && other.is_number()) {
    return as_number() == other.as_number();
  }
  if (is_list() && other.is_list()) {
    return as_list() == other.as_list();
  }
  if (is_map() && other.is_map()) {
    return as_map() == other.as_map();
  }
  if (data_.index() != other.data_.index()) {
    return false;
  }
  return is_null() || as_string() == other.as_string();
}

Value
from_yaml(const YAML::Node & node)
{
  switch (node.Type()) {
    case YAML::NodeType::Sequence: {
        Value::List list;
        for (const auto & item : node) {
          list.push_back(from_yaml(item));
        }
        return Value(std::move(list));
      }
    case YAML::NodeType::Map: {
        Value::Map map;
        for (const auto & item : node) {
          map[item.first.Scalar()] = from_yaml(item.second);
        }
        return Value(std::move(map));
      }
    case YAML::NodeType::Scalar: {
        const std::string & text = node.Scalar();
        if (node.Tag() == "!") {  // Quoted: a string, whatever it looks like
          return Value(text);
        }
        if (text == "true" || text == "True" || text == "TRUE") {
          return Value(true);
        }
        if (text == "false" || text == "False" || text == "FALSE") {
          return Value(false);
        }
        int64_t integer = 0;
        auto [iend, ierror] = std::from_chars(text.data(), text.data() + text.size(), integer);
        if (ierror == std::errc() && iend == text.data() + text.size()) {
          return Value(integer);
        }
        double real = 0.0;
        auto [rend, rerror] = std::from_chars(text.data(), text.data() + text.size(), real);
        if (rerror == std::errc() && rend == text.data() + text.size()) {
          return Value(real);
        }
        if (text == "~" || text == "null" || text == "Null" || text == "NULL") {
          return Value();
        }
        return Value(text);
      }
    default:
      return Value();
  }
}

std::string
format_g(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%g", value);
  return buffer;
}

}  // namespace plansys2_state_updater
