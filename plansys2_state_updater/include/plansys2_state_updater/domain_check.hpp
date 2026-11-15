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

#ifndef PLANSYS2_STATE_UPDATER__DOMAIN_CHECK_HPP_
#define PLANSYS2_STATE_UPDATER__DOMAIN_CHECK_HPP_

#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2_state_updater
{

class Engine;

/// The parameters of a predicate or function: (type, its subtypes) each
using Signature = std::vector<std::pair<std::string, std::vector<std::string>>>;

/// What the checks on configure need of the domain and the problem
struct DomainInfo
{
  std::map<std::string, Signature> predicates;
  std::map<std::string, Signature> functions;
  // The functions whose value is an object (object fluents), with its type: (pos ?r) - place
  std::map<std::string, std::string> object_fluents;
  // What the effects of the actions change
  std::set<std::string> modified_predicates;
  std::set<std::string> modified_functions;
  // Objects of the problem and constants of the domain: name -> type (nullopt: unknown)
  std::optional<std::map<std::string, std::string>> objects;
  std::optional<std::set<std::string>> measured_functions;  // nullopt: unknown
  std::optional<std::set<std::string>> types;
};

/// The predicates and functions that some effects change (not those in the condition of a when)
std::pair<std::set<std::string>, std::set<std::string>> modified_by(
  const std::vector<plansys2_msgs::msg::Tree> & effects);

/// The errors and the warnings of the rules against the domain and the problem; a predicate
/// the actions change is an error: it would fight the executor
std::pair<std::vector<std::string>, std::vector<std::string>> check_domain(
  const Engine & engine, const DomainInfo & domain);

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__DOMAIN_CHECK_HPP_
