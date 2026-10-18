// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cctype>
#include <set>

#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainReader.hpp"


namespace plansys2
{

namespace
{

// The top level expressions of a block, line by line as written. One that takes several
// lines is joined into one, so a predicate written over several lines is still one (#286)
std::vector<std::string> split_expressions(const std::string & block)
{
  std::vector<std::string> ret;
  std::string current;
  int depth = 0;
  for (const auto & line : tokenize(block, "\n")) {
    if (depth == 0) {
      current = line;
    } else {
      auto start = line.find_first_not_of(" \t\r");
      if (start != std::string::npos) {
        current += " " + line.substr(start);
      }
    }
    for (char c : line) {
      depth += (c == '(') - (c == ')');
    }
    if (depth <= 0) {
      depth = 0;
      if (current.find_first_not_of(" \t\r") != std::string::npos) {
        ret.push_back(current);
      }
    }
  }
  return ret;
}

// The same expression whatever its blanks
std::string expression_key(const std::string & expression)
{
  std::string collapsed;
  for (char c : expression) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!collapsed.empty() && collapsed.back() != ' ') {
        collapsed += ' ';
      }
    } else {
      collapsed += c;
    }
  }
  std::string ret;
  for (size_t i = 0; i < collapsed.size(); i++) {
    const char c = collapsed[i];
    const bool after_open = !ret.empty() && ret.back() == '(';
    const bool before_close = i + 1 < collapsed.size() && collapsed[i + 1] == ')';
    if (c == ' ' && (ret.empty() || after_open || before_close)) {
      continue;
    }
    ret += c;
  }
  return ret;
}

}  // namespace

DomainReader::DomainReader()
{
}

void
DomainReader::add_domain(const std::string & domain)
{
  if (domain.empty()) {
    std::cerr << "Empty domain" << std::endl;
    return;
  }

  Domain new_domain;

  std::string lc_domain = domain;
  std::transform(
    domain.begin(), domain.end(), lc_domain.begin(),
    [](unsigned char c) {return std::tolower(c);});

  lc_domain = remove_comments(lc_domain);

  new_domain.name = get_name(lc_domain);
  new_domain.requirements = get_requirements(lc_domain);
  new_domain.types = get_types(lc_domain);
  new_domain.constants = get_constants(lc_domain);
  new_domain.predicates = get_predicates(lc_domain);
  new_domain.functions = get_functions(lc_domain);
  new_domain.derived_predicates = get_derived_predicates(lc_domain);
  new_domain.actions = get_actions(lc_domain);
  new_domain.happenings = get_happenings(lc_domain);
  new_domain.constraints = get_constraints(lc_domain);

  domains_.push_back(new_domain);
}

std::string
DomainReader::get_joint_domain() const
{
  std::string ret = "(define (domain ";

  for (size_t i = 0; i < domains_.size(); i++) {
    ret += domains_[i].name;
    if (i < domains_.size() - 1) {
      ret += "_";
    }
  }
  ret += ")\n";

  ret += "(:requirements ";

  std::set<std::string> reqs_set;
  for (auto & domain : domains_) {
    std::vector<std::string> reqs = tokenize(domain.requirements, " ");
    reqs_set.insert(reqs.begin(), reqs.end());
  }
  for (auto & req : reqs_set) {
    ret += req + " ";
  }
  ret += ")\n\n";

  bool functions = false;
  bool constants = false;
  bool types = false;

  for (auto & domain : domains_) {
    if (!domain.functions.empty()) {
      functions = true;
    }
    if (!domain.types.empty()) {
      types = true;
    }
    if (!domain.constants.empty()) {
      constants = true;
    }
  }

  if (types) {
    ret += "(:types\n";
    for (auto & domain : domains_) {
      if (!domain.types.empty()) {
        ret += domain.types + "\n";
      }
    }
    ret += ")\n\n";
  }

  if (constants) {
    ret += "(:constants\n";
    for (const auto & domain : domains_) {
      if (!domain.constants.empty()) {
        ret += domain.constants + "\n";
      }
    }
    ret += ")\n\n";
  }

  ret += "(:predicates\n";
  // Each predicate once, a whole expression even if it was written over several lines
  std::set<std::string> preds_set;
  std::set<std::string> preds_seen;
  for (auto & domain : domains_) {
    for (const auto & pred : split_expressions(domain.predicates)) {
      if (preds_seen.insert(expression_key(pred)).second) {
        preds_set.insert(pred);
      }
    }
  }
  for (auto & pred : preds_set) {
    ret += pred + "\n";
  }
  ret += ")\n\n";

  if (functions) {
    ret += "(:functions\n";
    for (auto & domain : domains_) {
      if (!domain.functions.empty()) {
        ret += domain.functions + "\n";
      }
    }
    ret += ")\n\n";
  }

  for (auto & domain : domains_) {
    for (auto & derived : domain.derived_predicates) {
      if (!derived.empty()) {
        ret += derived + "\n\n";
      }
    }
  }

  for (auto & domain : domains_) {
    for (auto & action : domain.actions) {
      if (!action.empty()) {
        ret += action + "\n";
      }
    }
  }

  for (auto & domain : domains_) {
    for (auto & happening : domain.happenings) {
      ret += happening + "\n";
    }
  }

  // Those of every domain hold
  std::vector<std::string> constraints;
  for (auto & domain : domains_) {
    if (!domain.constraints.empty()) {
      constraints.push_back(domain.constraints);
    }
  }
  if (constraints.size() == 1) {
    ret += "(:constraints " + constraints[0] + ")\n";
  } else if (constraints.size() > 1) {
    ret += "(:constraints (and";
    for (const auto & c : constraints) {
      ret += " " + c;
    }
    ret += "))\n";
  }

  ret += ")\n";

  return ret;
}

std::string
DomainReader::get_constraints(const std::string & domain)
{
  const std::string head = "(:constraints";
  const auto pos = domain.find(head);
  if (pos == std::string::npos) {
    return {};
  }
  const auto end_pos = get_end_block(domain, pos + 2);
  if (end_pos == -1) {
    return {};
  }
  auto ret = domain.substr(pos + head.size(), end_pos - pos - head.size());
  const auto first = ret.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  return ret.substr(first, ret.find_last_not_of(" \t\r\n") - first + 1);
}

int
DomainReader::get_end_block(const std::string & domain, std::size_t init_pos)
{
  std::size_t domain_length = domain.length();

  auto end_pos = init_pos;
  int p_counter = 1;
  while (end_pos < domain_length && p_counter > 0) {
    if (domain[end_pos] == '(') {
      p_counter++;
    } else if (domain[end_pos] == ')') {
      p_counter--;
    }

    if (p_counter > 0) {
      end_pos++;
    }
  }

  if (p_counter == 0) {
    return end_pos;
  } else {
    return -1;
  }
}

std::string
DomainReader::get_name(std::string & domain)
{
  const std::string pattern("domain");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    auto ret = substr_without_empty_lines(domain, init_pos, end_pos);

    // remove spaces
    ret.erase(std::remove(ret.begin(), ret.end(), ' '), ret.end());

    return ret;
  } else {
    return "";
  }
}

std::string
DomainReader::get_requirements(std::string & domain)
{
  const std::string pattern(":requirements");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    auto ret = substr_without_empty_lines(domain, init_pos, end_pos);
    // We remove the requirements part for not interfering with next analysis
    domain.replace(init_pos, end_pos - init_pos, "");

    return ret;
  } else {
    return "";
  }
}

std::string
DomainReader::get_types(const std::string & domain)
{
  const std::string pattern(":types");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}


std::string
DomainReader::get_constants(const std::string & domain)
{
  const std::string pattern(":constants");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}


std::string
DomainReader::get_predicates(const std::string & domain)
{
  const std::string pattern(":predicates");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}

std::string
DomainReader::get_functions(const std::string & domain)
{
  const std::string pattern(":functions");

  std::size_t init_pos = domain.find(pattern);
  if (init_pos == std::string::npos) {
    return "";
  }
  init_pos += pattern.length();

  auto end_pos = get_end_block(domain, init_pos);

  if (end_pos >= 0) {
    std::string ret = substr_without_empty_lines(domain, init_pos, end_pos);
    return ret;
  } else {
    return "";
  }
}

std::vector<std::string>
DomainReader::get_derived_predicates(const std::string & domain)
{
  std::vector<std::string> ret;

  const std::string derived_pattern(":derived");

  auto ldomain = domain;

  size_t pos = std::string::npos;
  do {
    pos = ldomain.find(derived_pattern);

    if (pos != std::string::npos) {
      auto end_pos = get_end_block(ldomain, pos + 1);

      if (end_pos == -1) {
        break;
      }

      std::string lines = substr_without_empty_lines(ldomain, pos, end_pos + 1);
      ret.push_back("(" + lines);
      ldomain = ldomain.substr(end_pos + 1);
    }
  } while (!ldomain.empty() && pos != std::string::npos);

  return ret;
}

std::vector<std::string>
DomainReader::get_actions(const std::string & domain)
{
  std::vector<std::string> ret;

  const std::string da_pattern(":durative-action");
  const std::string a_pattern(":action");

  auto ldomain = domain;

  size_t pos = std::string::npos;
  do {
    std::size_t da_pos = ldomain.find(da_pattern);
    std::size_t a_pos = ldomain.find(a_pattern);

    pos = std::min(da_pos, a_pos);

    if (pos != std::string::npos) {
      auto end_pos = get_end_block(ldomain, pos + 1);

      if (end_pos == -1) {
        break;
      }

      std::string lines = substr_without_empty_lines(ldomain, pos, end_pos + 1);
      ret.push_back("(" + lines);
      ldomain = ldomain.substr(end_pos + 1);
    }
  } while (!ldomain.empty() && pos != std::string::npos);

  return ret;
}


std::vector<std::string>
DomainReader::get_happenings(const std::string & domain)
{
  std::vector<std::string> ret;
  auto ldomain = domain;
  while (true) {
    const auto pos = std::min(ldomain.find("(:process"), ldomain.find("(:event"));
    if (pos == std::string::npos) {
      break;
    }
    auto end_pos = get_end_block(ldomain, pos + 2);
    if (end_pos == -1) {
      break;
    }
    ret.push_back(substr_without_empty_lines(ldomain, pos, end_pos + 1));
    ldomain = ldomain.substr(end_pos + 1);
  }
  return ret;
}

}  // namespace plansys2
