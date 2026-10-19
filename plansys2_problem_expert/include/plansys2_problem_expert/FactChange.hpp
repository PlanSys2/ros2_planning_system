// Copyright 2026 Intelligent Robotics Lab
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

#ifndef PLANSYS2_PROBLEM_EXPERT__FACTCHANGE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__FACTCHANGE_HPP_

#include <cstdint>

#include <string>

#include "plansys2_msgs/msg/node.hpp"

namespace plansys2
{

/**
 * @brief A change an applied effect made to one fact, to take it back later.
 */
struct FactChange
{
  plansys2_msgs::msg::Node fact;  ///< The grounded predicate or function
  bool existed_before = false;    ///< Predicates: whether it held before the effect
  double value_before = 0.0;      ///< Functions: value before the effect
  double value_after = 0.0;       ///< Functions: value the effect left
  uint8_t modifier_type = 0;      ///< Functions: the modifier that changed it
  std::string object_before;      ///< Object fluents: object before the effect (empty: none)
  std::string object_after;       ///< Object fluents: object the effect left
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__FACTCHANGE_HPP_
