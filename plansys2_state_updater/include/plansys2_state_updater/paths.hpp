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

#ifndef PLANSYS2_STATE_UPDATER__PATHS_HPP_
#define PLANSYS2_STATE_UPDATER__PATHS_HPP_

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_state_updater/value.hpp"

namespace plansys2_state_updater
{

/// One step of a path: a field by name, an index, or [*], each element
struct PathStep
{
  enum Kind {FIELD, INDEX, WILDCARD} kind {FIELD};
  std::string name;
  size_t index {0};

  bool operator==(const PathStep & other) const
  {
    return kind == other.kind && name == other.name && index == other.index;
  }
};

using Path = std::vector<PathStep>;

/**
 * @brief Split 'cells[2].voltage' in fields and indexes; '[*]' at most once; '' is the message.
 * @throws std::invalid_argument if it is not a path.
 */
Path parse_path(const std::string & path);

bool has_wildcard(const Path & path);

/// What the path is on, without the steps after [*]: a.b[*].c is a.b
Path before_wildcard(const Path & path);
/// The steps after [*]: a.b[*].c is c
Path after_wildcard(const Path & path);

/**
 * @brief Follow a path without [*] in a value (a message, as a map).
 * @throws std::out_of_range if a field or an index is not there.
 */
Value get_field(const Value & message, const Path & path);

/// (element, value) for each element a path reaches: one per entry of status in 'status[*].level'
std::vector<std::pair<Value, Value>> get_items(const Value & message, const Path & path);

/**
 * @brief Name and parameters of '(name p1 p2)'; a parameter may be a placeholder, '{name}'.
 * @throws std::invalid_argument if it is not a fact.
 */
std::pair<std::string, std::vector<std::string>> parse_fact(const std::string & fact);

/// A fact as '(name p1 p2)', whatever its spaces, to compare it with others
std::string canonical(const std::string & fact);

/// Whether a text is a PDDL name: a letter, then letters, digits, '_' and '-'
bool is_name(const std::string & text);

/// A value as a PDDL name; std::invalid_argument if it is not one
std::string as_name(const Value & value, const std::string & fact);

/// The predicates and the numeric functions of a plansys2_msgs/Knowledge; object fluents are left
/// out: no rule writes them
std::pair<std::set<std::string>, std::map<std::string, double>> parse_knowledge(
  const std::vector<std::string> & predicates, const std::vector<std::string> & functions);

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__PATHS_HPP_
