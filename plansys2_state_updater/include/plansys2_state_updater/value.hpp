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

#ifndef PLANSYS2_STATE_UPDATER__VALUE_HPP_
#define PLANSYS2_STATE_UPDATER__VALUE_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace YAML
{
class Node;
}  // namespace YAML

namespace plansys2_state_updater
{

/// A message read by introspection, or a piece of YAML; lists and maps are shared: a copy is cheap
class Value
{
public:
  using List = std::vector<Value>;
  using Map = std::map<std::string, Value>;

  Value() = default;
  Value(bool value);  // NOLINT(runtime/explicit): a value is what it holds
  Value(const char * value);  // NOLINT(runtime/explicit)
  Value(std::string value);  // NOLINT(runtime/explicit)
  Value(List value);  // NOLINT(runtime/explicit)
  Value(Map value);  // NOLINT(runtime/explicit)
  template<typename T,
    std::enable_if_t<std::is_integral_v<T>&& !std::is_same_v<T, bool>, int> = 0>
  Value(T value)  // NOLINT(runtime/explicit)
  : data_(static_cast<int64_t>(value)) {}
  template<typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
  Value(T value)  // NOLINT(runtime/explicit)
  : data_(static_cast<double>(value)) {}

  bool is_null() const {return std::holds_alternative<std::monostate>(data_);}
  bool is_bool() const {return std::holds_alternative<bool>(data_);}
  bool is_int() const {return std::holds_alternative<int64_t>(data_);}
  bool is_double() const {return std::holds_alternative<double>(data_);}
  // A boolean too, as 1 or 0
  bool is_number() const {return is_bool() || is_int() || is_double();}
  bool is_string() const {return std::holds_alternative<std::string>(data_);}
  bool is_list() const {return std::holds_alternative<std::shared_ptr<const List>>(data_);}
  bool is_map() const {return std::holds_alternative<std::shared_ptr<const Map>>(data_);}

  // The value as it is; std::invalid_argument if it is not of that kind
  bool as_bool() const;
  double as_number() const;
  const std::string & as_string() const;
  const List & as_list() const;
  const Map & as_map() const;
  // Truthy, as Python: false, 0, "", an empty list or map, and null are false
  bool truthy() const;

  // As Python's str(): 7, 2.0, 0.1, text as it is; true and false; lists and maps as JSON
  std::string text() const;
  std::string json() const;
  // What kind it is, for errors: "a number", "a string"...
  std::string kind() const;

  bool operator==(const Value & other) const;
  bool operator!=(const Value & other) const {return !(*this == other);}

private:
  std::variant<std::monostate, bool, int64_t, double, std::string,
    std::shared_ptr<const List>, std::shared_ptr<const Map>> data_;
};

/// A piece of YAML as a value: quoted scalars are strings; plain ones, what they look like
Value from_yaml(const YAML::Node & node);

/// A real as Python's '%g', for the reasons: 95, 0.5, 1e+06
std::string format_g(double value);

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__VALUE_HPP_
