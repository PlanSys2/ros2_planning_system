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

#include "plansys2_state_updater/problem_writer.hpp"

#include <cctype>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <variant>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_state_updater/paths.hpp"

namespace plansys2_state_updater
{

using plansys2_msgs::msg::ProblemChange;

namespace
{

std::string
function_text(const Change & change)
{
  const std::string value = std::holds_alternative<std::string>(change.value) ?
    std::get<std::string>(change.value) : Value(std::get<double>(change.value)).text();
  return "(= " + change.fact + " " + value + ")";
}

bool
is_true(const FactValue & value)
{
  return std::holds_alternative<bool>(value) && std::get<bool>(value);
}

// Whole word: contact_7 is not in (at contact_70)
std::regex
whole_word(const std::string & name)
{
  std::string escaped;
  for (char c : name) {
    escaped += (std::isalnum(static_cast<unsigned char>(c)) || c == '_') ? std::string(1, c) :
      std::string("\\") + c;
  }
  return std::regex("[\\s(]" + escaped + "[\\s)]", std::regex::icase);
}

}  // namespace

ProblemWriter::ProblemWriter(const plansys2::ProblemAccess & access)
: access_(access)
{
}

void
ProblemWriter::start(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node_ = node;
  plan_client_ = node->create_client<plansys2_msgs::srv::GetPlan>("executor/get_remaining_plan");
  plan_ = plansys2_msgs::msg::Plan();
  last_ask_ = -1e9;
}

void
ProblemWriter::stop()
{
  plan_client_.reset();
}

ProblemChange
ProblemWriter::problem_change(const Change & change)
{
  ProblemChange out;
  if (change.kind == "instance") {
    const bool create = std::holds_alternative<std::string>(change.value);
    out.type = create ? ProblemChange::ADD_INSTANCE : ProblemChange::REMOVE_INSTANCE;
    out.instance.name = change.fact;
    out.instance.type = create ? std::get<std::string>(change.value) : "";
  } else if (change.kind == "function") {
    out.type = ProblemChange::SET_FUNCTION;
    out.node = parser::pddl::fromStringFunction(function_text(change));
  } else {
    out.type = is_true(change.value) ? ProblemChange::ADD_PREDICATE :
      ProblemChange::REMOVE_PREDICATE;
    out.node = parser::pddl::fromStringPredicate(change.fact);
  }
  return out;
}

std::vector<bool>
ProblemWriter::write(const std::vector<Change> & changes)
{
  if (changes.empty()) {
    return {};
  }
  std::vector<ProblemChange> batch;
  for (const auto & change : changes) {
    batch.push_back(problem_change(change));
  }
  std::string error;
  std::vector<bool> results;
  if (access_.problem->applyChanges(batch, error)) {
    results.assign(changes.size(), true);
  } else {
    // One by one, so a change that cannot be made does not hold back the others
    for (const auto & change : batch) {
      results.push_back(access_.problem->applyChanges({change}, error));
    }
  }
  // A single notification, so a single replanning
  for (bool made : results) {
    if (made) {
      access_.notify();
      break;
    }
  }
  return results;
}

std::set<std::string>
ProblemWriter::predicates() const
{
  std::set<std::string> holding;
  for (const auto & predicate : access_.problem->getPredicates()) {
    holding.insert(canonical(parser::pddl::toString(predicate)));
  }
  return holding;
}

std::map<std::string, double>
ProblemWriter::functions() const
{
  std::map<std::string, double> values;
  for (const auto & function : access_.problem->getFunctions()) {
    if (function.object.empty()) {
      values[canonical(parser::pddl::toString(function))] = function.value;
    }
  }
  return values;
}

std::map<std::string, std::string>
ProblemWriter::objects() const
{
  std::map<std::string, std::string> objects;
  for (const auto & function : access_.problem->getFunctions()) {
    if (!function.object.empty()) {
      objects[canonical(parser::pddl::toString(function))] = function.object;
    }
  }
  return objects;
}

std::set<std::string>
ProblemWriter::instances() const
{
  std::set<std::string> names;
  for (const auto & instance : access_.problem->getInstances()) {
    names.insert(instance.name);
  }
  return names;
}

std::optional<double>
ProblemWriter::current_value(const std::string & fact) const
{
  const auto function = access_.problem->getFunction(fact);
  if (!function || !function->object.empty()) {
    return std::nullopt;
  }
  return function->value;
}

bool
ProblemWriter::in_use(const std::string & name)
{
  const std::regex uses = whole_word(name);
  if (std::regex_search(parser::pddl::toString(access_.problem->getGoal()), uses)) {
    return true;
  }
  ask_plan();
  for (const auto & item : plan_.items) {
    if (std::regex_search(item.action, uses)) {
      return true;
    }
  }
  return false;
}

void
ProblemWriter::ask_plan()
{
  auto node = node_.lock();
  if (!node || !plan_client_) {
    return;
  }
  if (!plan_client_->service_is_ready()) {
    plan_ = plansys2_msgs::msg::Plan();
    return;
  }
  // At most once a second, never waited for: the answer arrives in a callback
  const double now = node->now().seconds();
  if (last_ask_ > now) {
    last_ask_ = now - 1.0;  // The clock went back
  }
  if (now - last_ask_ < 1.0) {
    return;
  }
  last_ask_ = now;
  plan_client_->prune_pending_requests();
  plan_client_->async_send_request(
    std::make_shared<plansys2_msgs::srv::GetPlan::Request>(),
    [this](rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedFuture future) {
      const auto response = future.get();
      plan_ = response->success ? response->plan : plansys2_msgs::msg::Plan();
    });
}

}  // namespace plansys2_state_updater
