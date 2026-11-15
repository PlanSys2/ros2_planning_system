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

#include "plansys2_state_updater/domain_reader.hpp"

#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace plansys2_state_updater
{

namespace
{

// The lists lack the parameter types: they come with the details
template<typename NodeT, typename GetDetails>
std::map<std::string, Signature>
signatures(const std::vector<NodeT> & nodes, GetDetails get_details)
{
  std::map<std::string, Signature> out;
  for (const auto & node : nodes) {
    const auto details = get_details(node.name);
    const auto & parameters = details ? details->parameters : node.parameters;
    Signature signature;
    for (const auto & param : parameters) {
      signature.emplace_back(param.type, param.sub_types);
    }
    out[node.name] = signature;
  }
  return out;
}

}  // namespace

DomainInfo
read_domain(
  plansys2::DomainExpertInterface & domain, plansys2::ProblemExpert & problem,
  const std::optional<std::set<std::string>> & measured_functions)
{
  DomainInfo info;
  info.predicates = signatures(
    domain.getPredicates(), [&](const std::string & name) {return domain.getPredicate(name);});
  info.functions = signatures(
    domain.getFunctions(), [&](const std::string & name) {return domain.getFunction(name);});
  for (const auto & [name, signature] : info.functions) {
    (void)signature;
    const auto function = domain.getFunction(name);
    if (function && !function->object.empty()) {
      info.object_fluents[name] = function->object;
    }
  }

  // What the plan itself changes: the state updater must not fight it
  std::vector<plansys2_msgs::msg::Tree> effects;
  for (const auto & name : domain.getActions()) {
    if (auto action = domain.getAction(name, {})) {
      effects.push_back(action->effects);
    }
  }
  for (const auto & name : domain.getProcesses()) {
    if (auto process = domain.getProcess(name, {})) {
      effects.push_back(process->effects);
    }
  }
  for (const auto & name : domain.getEvents()) {
    if (auto event = domain.getEvent(name, {})) {
      effects.push_back(event->effects);
    }
  }
  for (const auto & name : domain.getDurativeActions()) {
    if (auto action = domain.getDurativeAction(name, {})) {
      effects.push_back(action->at_start_effects);
      effects.push_back(action->at_end_effects);
      effects.push_back(action->continuous_effects);
    }
  }
  std::tie(info.modified_predicates, info.modified_functions) = modified_by(effects);

  std::map<std::string, std::string> objects;
  for (const auto & instance : problem.getInstances()) {
    objects[instance.name] = instance.type;
  }
  std::set<std::string> types;
  for (const auto & type : domain.getTypes()) {
    types.insert(type);
    for (const auto & constant : domain.getConstants(type)) {
      objects.emplace(constant, type);
    }
  }
  info.objects = objects;
  info.types = types;
  info.measured_functions = measured_functions;
  return info;
}

}  // namespace plansys2_state_updater
