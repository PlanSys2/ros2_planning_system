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

#include "plansys2_state_updater/paths.hpp"

#include <algorithm>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace plansys2_state_updater
{

namespace
{

const std::regex kFact(R"(^\(\s*([^\s()]+)((?:\s+[^\s()]+)*)\s*\)$)");
const std::regex kPart(R"(^([A-Za-z_][A-Za-z0-9_]*)((?:\[(?:\d+|\*)\])*)$)");
const std::regex kIndex(R"(\[(\d+|\*)\])");
const std::regex kFunctionValue(R"(^\(\s*=\s*(\(.*\))\s+(\S+)\s*\)$)");

std::vector<std::string>
split(const std::string & text, char separator)
{
  std::vector<std::string> parts;
  size_t start = 0;
  for (size_t end = text.find(separator); end != std::string::npos;
    end = text.find(separator, start))
  {
    parts.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  parts.push_back(text.substr(start));
  return parts;
}

std::string
trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return "";
  }
  return text.substr(first, text.find_last_not_of(" \t\n\r") - first + 1);
}

}  // namespace

Path
parse_path(const std::string & path)
{
  Path steps;
  if (path.empty()) {
    return steps;
  }
  for (const auto & part : split(path, '.')) {
    std::smatch match;
    if (!std::regex_match(part, match, kPart)) {
      throw std::invalid_argument("'" + path + "' is not a field path like 'cells[2].voltage'");
    }
    steps.push_back(PathStep{PathStep::FIELD, match[1].str(), 0});
    const std::string indexes = match[2].str();
    for (std::sregex_iterator it(indexes.begin(), indexes.end(), kIndex), end; it != end; ++it) {
      const std::string index = (*it)[1].str();
      if (index == "*") {
        steps.push_back(PathStep{PathStep::WILDCARD, "", 0});
      } else {
        steps.push_back(PathStep{PathStep::INDEX, "", std::stoul(index)});
      }
    }
  }
  if (std::count_if(
      steps.begin(), steps.end(),
      [](const PathStep & s) {return s.kind == PathStep::WILDCARD;}) > 1)
  {
    throw std::invalid_argument("'" + path + "' has more than one [*]");
  }
  return steps;
}

bool
has_wildcard(const Path & path)
{
  return std::any_of(
    path.begin(), path.end(), [](const PathStep & s) {return s.kind == PathStep::WILDCARD;});
}

Path
before_wildcard(const Path & path)
{
  auto it = std::find_if(
    path.begin(), path.end(), [](const PathStep & s) {return s.kind == PathStep::WILDCARD;});
  return Path(path.begin(), it);
}

Path
after_wildcard(const Path & path)
{
  auto it = std::find_if(
    path.begin(), path.end(), [](const PathStep & s) {return s.kind == PathStep::WILDCARD;});
  return it == path.end() ? Path{} : Path(it + 1, path.end());
}

Value
get_field(const Value & message, const Path & path)
{
  // By reference down the path, one copy at the end: it is done for every element
  const Value * value = &message;
  for (const auto & step : path) {
    if (step.kind == PathStep::INDEX) {
      if (!value->is_list() || step.index >= value->as_list().size()) {
        throw std::out_of_range("no [" + std::to_string(step.index) + "]");
      }
      value = &value->as_list()[step.index];
    } else {
      if (!value->is_map()) {
        throw std::out_of_range("no field '" + step.name + "' in " + value->kind());
      }
      const auto & map = value->as_map();
      auto it = map.find(step.name);
      if (it == map.end()) {
        throw std::out_of_range("no field '" + step.name + "'");
      }
      value = &it->second;
    }
  }
  return *value;
}

std::vector<std::pair<Value, Value>>
get_items(const Value & message, const Path & path)
{
  if (!has_wildcard(path)) {
    return {{message, get_field(message, path)}};
  }
  const Value elements = get_field(message, before_wildcard(path));
  if (!elements.is_list()) {
    throw std::out_of_range("[*] on " + elements.kind() + ", not a list");
  }
  const Path after = after_wildcard(path);
  std::vector<std::pair<Value, Value>> items;
  for (const auto & element : elements.as_list()) {
    items.emplace_back(element, get_field(element, after));
  }
  return items;
}

std::pair<std::string, std::vector<std::string>>
parse_fact(const std::string & fact)
{
  std::smatch match;
  const std::string text = trim(fact);
  if (!std::regex_match(text, match, kFact)) {
    throw std::invalid_argument("'" + fact + "' is not a fact like '(name p1 p2)'");
  }
  std::vector<std::string> params;
  std::string rest = match[2].str();
  size_t pos = 0;
  while ((pos = rest.find_first_not_of(" \t\n\r", pos)) != std::string::npos) {
    const size_t end = rest.find_first_of(" \t\n\r", pos);
    params.push_back(rest.substr(pos, end - pos));
    pos = end;
  }
  return {match[1].str(), params};
}

std::string
canonical(const std::string & fact)
{
  const auto [name, params] = parse_fact(fact);
  std::string text = "(" + name;
  for (const auto & param : params) {
    text += " " + param;
  }
  return text + ")";
}

bool
is_name(const std::string & text)
{
  // [A-Za-z][A-Za-z0-9_-]*, without a regex: it is checked for every element of every message
  auto letter = [](char c) {return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');};
  if (text.empty() || !letter(text[0])) {
    return false;
  }
  return std::all_of(
    text.begin() + 1, text.end(), [&letter](char c) {
      return letter(c) || (c >= '0' && c <= '9') || c == '_' || c == '-';
    });
}

std::string
as_name(const Value & value, const std::string & fact)
{
  const std::string name = value.text();
  if (!value.is_string() && !value.is_int()) {
    throw std::invalid_argument("'" + name + "' is not a valid PDDL name, for " + fact);
  }
  if (!is_name(name)) {
    throw std::invalid_argument("'" + name + "' is not a valid PDDL name, for " + fact);
  }
  return name;
}

std::pair<std::set<std::string>, std::map<std::string, double>>
parse_knowledge(
  const std::vector<std::string> & predicates, const std::vector<std::string> & functions)
{
  std::set<std::string> holding;
  for (const auto & predicate : predicates) {
    try {
      holding.insert(canonical(predicate));
    } catch (const std::invalid_argument &) {
    }
  }
  std::map<std::string, double> values;
  for (const auto & function : functions) {
    std::smatch match;
    const std::string text = trim(function);
    if (!std::regex_match(text, match, kFunctionValue)) {
      continue;
    }
    try {
      values[canonical(match[1].str())] = Value(match[2].str()).as_number();
    } catch (const std::invalid_argument &) {
    }
  }
  return {holding, values};
}

}  // namespace plansys2_state_updater
