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

#include <algorithm>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "plansys2_state_updater/paths.hpp"
#include "plansys2_state_updater/transforms.hpp"

namespace plansys2_state_updater
{

/// Lowest value of a field over a list, clamped to an optional minimum.
/// args: {field: voltage, minimum: 3.0}
class MinOf : public TransformPlugin
{
public:
  Value apply(const Value & value, const Value & args) override
  {
    const auto & map = args.as_map();
    const Path field = map.count("field") ? parse_path(map.at("field").text()) : Path{};
    const auto & list = value.as_list();
    if (list.empty()) {
      throw std::invalid_argument("an empty list has no lowest");
    }
    double lowest = get_field(list[0], field).as_number();
    for (const auto & item : list) {
      lowest = std::min(lowest, get_field(item, field).as_number());
    }
    if (map.count("minimum")) {
      lowest = std::max(lowest, map.at("minimum").as_number());
    }
    return Value(lowest);
  }
};

}  // namespace plansys2_state_updater

PLUGINLIB_EXPORT_CLASS(plansys2_state_updater::MinOf, plansys2_state_updater::TransformPlugin)
