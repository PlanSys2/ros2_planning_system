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

#include "plansys2_core/PddlRequirements.hpp"

#include <cctype>
#include <charconv>
#include <set>
#include <string>
#include <vector>

namespace plansys2
{

namespace
{

// Parentheses and words, lowercase, without comments
std::vector<std::string> tokenize(const std::string & text)
{
  std::vector<std::string> tokens;
  std::string word;
  auto flush = [&]() {
      if (!word.empty()) {
        tokens.push_back(word);
        word.clear();
      }
    };
  for (size_t i = 0; i < text.size(); i++) {
    const char c = text[i];
    if (c == ';') {
      flush();
      while (i < text.size() && text[i] != '\n') {
        i++;
      }
    } else if (c == '(' || c == ')') {
      flush();
      tokens.emplace_back(1, c);
    } else if (std::isspace(static_cast<unsigned char>(c))) {
      flush();
    } else {
      word += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
  }
  flush();
  return tokens;
}

bool is_number(const std::string & token)
{
  double value = 0.0;
  const auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), value);
  return ec == std::errc() && ptr == token.data() + token.size();
}

bool is_comparison(const std::string & token)
{
  return token == "<=" || token == ">=" || token == "<" || token == ">";
}

// The type of a function, after its "-": number, or an object type
bool is_object_type(const std::string & token)
{
  return token != "number" && token != "(";
}

void scan(
  const std::string & text, bool problem, std::set<std::string> & needed,
  std::set<std::string> & declared)
{
  const auto tokens = tokenize(text);
  auto at = [&](size_t i) -> const std::string & {
      static const std::string none;
      return i < tokens.size() ? tokens[i] : none;
    };

  int depth = 0;
  int condition_depth = -1;  // Inside a precondition, condition or goal
  bool condition_next = false;
  int functions_depth = -1;  // Directly inside (:functions ...)
  int untyped_functions = 0;  // Declared since the last "- type"
  int init_depth = -1;  // Directly inside (:init ...)
  int requirements_depth = -1;

  for (size_t i = 0; i < tokens.size(); i++) {
    const auto & token = tokens[i];
    if (token == "(") {
      if (condition_next) {
        condition_depth = depth;
        condition_next = false;
      }
      depth++;
      if (depth == functions_depth + 1 && functions_depth >= 0) {
        untyped_functions++;  // Typed by the next "- type", numeric without one
      }
      const auto & head = at(i + 1);
      if (head == ":requirements") {
        requirements_depth = depth;
      } else if (head == ":functions") {
        functions_depth = depth;
      } else if (head == ":init" && problem) {
        init_depth = depth;
      } else if (head == ":derived") {
        needed.insert(":derived-predicates");
      } else if (head == "when") {
        needed.insert(":conditional-effects");
      } else if (head == ":process" || head == ":event" || head == ":constraint") {
        needed.insert(":time");
      } else if (head == ":constraints") {
        needed.insert(":constraints");
      } else if (head == "preference") {
        needed.insert(":preferences");
      } else if (head == ":durative-action") {
        needed.insert(":durative-actions");
      } else if (head == "or" || head == "imply") {
        needed.insert(":disjunctive-preconditions");
      } else if (head == "either") {
        needed.insert(":either-types");
      } else if (head == "exists") {
        needed.insert(":existential-preconditions");
      } else if (head == "forall") {
        needed.insert(":universal-preconditions");
      } else if (is_comparison(head) && at(i + 2) == "?duration") {
        needed.insert(":duration-inequalities");
      } else if (head == "not" && condition_depth >= 0 && at(i + 2) == "(" && at(i + 3) != "=") {
        needed.insert(":negative-preconditions");
      } else if (head == "at" && depth == init_depth + 1 && is_number(at(i + 2))) {
        // A value at a time is a timed initial fluent
        needed.insert(
          at(i + 3) == "(" && at(i + 4) == "=" ? ":timed-initial-fluents" :
          ":timed-initial-literals");
      }
    } else if (token == ")") {
      if (depth == requirements_depth) {
        requirements_depth = -1;
      }
      if (depth == functions_depth) {
        functions_depth = -1;
        if (untyped_functions > 0) {
          needed.insert(":numeric-fluents");
        }
        untyped_functions = 0;
      }
      if (depth == init_depth) {
        init_depth = -1;
      }
      depth--;
      if (depth == condition_depth) {
        condition_depth = -1;
      }
    } else if (depth == requirements_depth && token[0] == ':' && token != ":requirements") {
      declared.insert(token);
    } else if (token == ":precondition" || token == ":condition" || token == ":goal") {
      condition_next = true;
    } else if (token == "#t") {
      needed.insert(":continuous-effects");
    } else if (depth == functions_depth && token == "-") {
      if (untyped_functions > 0) {
        needed.insert(is_object_type(at(i + 1)) ? ":object-fluents" : ":numeric-fluents");
      }
      untyped_functions = 0;
    }
  }
}

}  // namespace

const std::set<std::string> & known_pddl_requirements()
{
  static const std::set<std::string> known = {
    ":strips", ":typing", ":negative-preconditions", ":disjunctive-preconditions", ":equality",
    ":existential-preconditions", ":universal-preconditions", ":quantified-preconditions",
    ":conditional-effects", ":fluents", ":numeric-fluents", ":object-fluents", ":adl",
    ":durative-actions", ":duration-inequalities", ":continuous-effects", ":derived-predicates",
    ":timed-initial-literals", ":preferences", ":constraints", ":action-costs", ":time",
    ":either-types", ":timed-initial-fluents"};
  return known;
}

std::set<std::string> pddl_requirements(const std::string & domain, const std::string & problem)
{
  // What the syntax shows, or every solver plans with
  static const std::set<std::string> shown = {
    ":strips", ":typing", ":equality", ":negative-preconditions", ":disjunctive-preconditions",
    ":existential-preconditions", ":universal-preconditions", ":quantified-preconditions",
    ":conditional-effects", ":fluents", ":numeric-fluents", ":object-fluents", ":adl",
    ":durative-actions", ":duration-inequalities", ":continuous-effects", ":derived-predicates",
    ":timed-initial-literals", ":preferences", ":constraints", ":time", ":either-types",
    ":timed-initial-fluents"};

  std::set<std::string> needed, declared;
  scan(domain, false, needed, declared);
  scan(problem, true, needed, declared);
  for (const auto & requirement : declared) {
    if (!shown.count(requirement)) {
      needed.insert(requirement);
    }
  }
  return needed;
}

std::vector<std::string> missing_requirements(
  const std::set<std::string> & needed, const std::set<std::string> & supported)
{
  std::vector<std::string> missing;
  if (supported.empty()) {
    return missing;
  }
  for (const auto & requirement : needed) {
    if (!supported.count(requirement)) {
      missing.push_back(requirement);
    }
  }
  return missing;
}

}  // namespace plansys2
