// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef PLANSYS2_BT_ACTIONS__JSONUTILS_HPP_
#define PLANSYS2_BT_ACTIONS__JSONUTILS_HPP_

#include <string>

#include "behaviortree_cpp/json_export.h"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/header.hpp"

// The follow templates are required when using Groot 2 to visualize the BT. They
// convert the data types into JSON format easy for visualization.

namespace builtin_interfaces::msg
{
BT_JSON_CONVERTER(builtin_interfaces::msg::Time, msg)
{
  add_field("sec", &msg.sec);
  add_field("nanosec", &msg.nanosec);
}

}  // namespace builtin_interfaces::msg

namespace std_msgs::msg
{

BT_JSON_CONVERTER(std_msgs::msg::Header, msg)
{
  add_field("stamp", &msg.stamp);
  add_field("frame_id", &msg.frame_id);
}

}  // namespace std_msgs::msg

namespace std
{

inline void from_json(const nlohmann::json & js, std::chrono::milliseconds & dest)
{
  if (js.contains("ms")) {
    dest = std::chrono::milliseconds(js.at("ms").get<int>());
  } else {
    throw std::runtime_error("Invalid JSON for std::chrono::milliseconds");
  }
}

inline void to_json(nlohmann::json & js, const std::chrono::milliseconds & src)
{
  js["__type"] = "std::chrono::milliseconds";
  js["ms"] = src.count();
}

}  // namespace std

#endif  // PLANSYS2_BT_ACTIONS__JSONUTILS_HPP_
