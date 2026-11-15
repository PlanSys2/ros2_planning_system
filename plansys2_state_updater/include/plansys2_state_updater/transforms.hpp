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

#ifndef PLANSYS2_STATE_UPDATER__TRANSFORMS_HPP_
#define PLANSYS2_STATE_UPDATER__TRANSFORMS_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_state_updater/paths.hpp"
#include "plansys2_state_updater/value.hpp"

namespace YAML
{
class Node;
}  // namespace YAML

namespace plansys2_state_updater
{

/// A transform of its own, as a pluginlib class named in the rules as 'my_package/MyTransform'
class TransformPlugin
{
public:
  virtual ~TransformPlugin() = default;

  /// The value of the fact (a number, a boolean or a name) from the field and the args (a map).
  /// @throws std::exception for a value it cannot take: an error of its rule, not of the node.
  virtual Value apply(const Value & value, const Value & args) = 0;
};

/// From the value of the field to the value of the fact
class Transform
{
public:
  /**
   * @brief From {kind: {args}} of the rules, or nothing (the field as it is).
   * @throws std::invalid_argument with what is wrong.
   */
  explicit Transform(const YAML::Node & spec);

  const std::string & kind() const {return kind_;}

  /// Whether it gives a boolean (true), a number (false), or what the field has (nullopt)
  std::optional<bool> gives_bool() const;

  /**
   * @brief Transform a value; active is the last output of a hysteresis, of the same fact.
   * @throws std::invalid_argument or std::out_of_range: an error of its rule.
   */
  Value apply(const Value & value, std::optional<bool> active = std::nullopt) const;

private:
  std::string kind_;
  Value args_;
  std::optional<Path> field_;
  std::vector<std::pair<Value, Value>> map_values_;
  std::shared_ptr<TransformPlugin> plugin_;
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__TRANSFORMS_HPP_
