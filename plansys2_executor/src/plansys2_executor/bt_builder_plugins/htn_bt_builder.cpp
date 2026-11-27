// Copyright 2025 Intelligent Robotics Lab
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

#include "plansys2_executor/bt_builder_plugins/htn_bt_builder.hpp"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

namespace
{

// The text of an XML attribute
std::string escaped(const std::string & text)
{
  std::string out;
  for (char c : text) {
    switch (c) {
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '&': out += "&amp;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

bool is_item(const std::string & ref) {return ref.rfind("item:", 0) == 0;}

size_t item_of(const std::string & ref) {return std::stoul(ref.substr(5));}

}  // namespace

void HTNBTBuilder::initialize(
  const std::string & bt_action_1, const std::string & bt_action_2, int precision)
{
  (void)bt_action_2;
  action_time_precision_ = precision;
  bt_action_ = !bt_action_1.empty() ? bt_action_1 :
    R""""(
<Sequence name="ACTION_ID">
WAIT_PREV_ACTIONS
  <WaitAtStartReq action="ACTION_ID"/>
  <ApplyAtStartEffect action="ACTION_ID"/>
  <ReactiveSequence name="ACTION_ID">
      <CheckOverAllReq action="ACTION_ID"/>
      <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
</Sequence>
)"""";
}

std::optional<std::vector<size_t>> HTNBTBuilder::total_order(
  size_t subtasks, const std::vector<uint32_t> & ordering)
{
  std::vector<std::vector<size_t>> after(subtasks);
  std::vector<size_t> pending(subtasks, 0);
  for (size_t k = 0; k + 1 < ordering.size(); k += 2) {
    if (ordering[k] >= subtasks || ordering[k + 1] >= subtasks) {
      return std::nullopt;
    }
    after[ordering[k]].push_back(ordering[k + 1]);
    ++pending[ordering[k + 1]];
  }
  // A total order is the one way to sort them: at each step a single subtask is free
  std::vector<size_t> order;
  std::vector<bool> done(subtasks, false);
  while (order.size() < subtasks) {
    std::optional<size_t> next;
    for (size_t s = 0; s < subtasks; ++s) {
      if (!done[s] && pending[s] == 0) {
        if (next) {
          return std::nullopt;  // two could go first
        }
        next = s;
      }
    }
    if (!next) {
      return std::nullopt;  // a cycle
    }
    done[*next] = true;
    order.push_back(*next);
    for (size_t b : after[*next]) {
      --pending[b];
    }
  }
  return order;
}

std::vector<size_t> HTNBTBuilder::ended_by(double time, size_t item) const
{
  // The same time: the times of the items are float32 (2e-6 apart at 25 s, 6e-5 at 1000 s) and
  // the plan's happenings 0.001 apart
  constexpr double kSame = 5e-4;
  const size_t n = plan_.items.size();
  auto end = [&](size_t j) {return plan_.items[j].time + plan_.items[j].duration;};
  std::vector<size_t> before;
  for (size_t j = 0; j < n; ++j) {
    if (j != item && end(j) <= time + kSame &&
      (item >= n || plan_.items[j].time < time - kSame || j < item))
    {
      before.push_back(j);
    }
  }
  std::vector<size_t> out;
  for (size_t j : before) {
    const bool implied = std::any_of(
      before.begin(), before.end(), [&](size_t k) {
        return k != j && plan_.items[k].time + kSame >= end(j) &&
               (plan_.items[k].time > end(j) + kSame || k > j);
      });
    if (!implied) {
      out.push_back(j);
    }
  }
  return out;
}

std::string HTNBTBuilder::waits_xml(const std::vector<size_t> & items, int depth) const
{
  std::string out;
  for (size_t before : items) {
    out += add_tabs(depth) + "<WaitAction action=\"" +
      escaped(to_action_id(plan_.items[before], action_time_precision_)) + "\"/>\n";
  }
  return out;
}

std::string HTNBTBuilder::action_xml(size_t item, int depth) const
{
  std::string text = bt_action_;
  replace(text, "ACTION_ID", escaped(to_action_id(plan_.items[item], action_time_precision_)));
  replace(text, "WAIT_PREV_ACTIONS", waits_xml(waits_[item], 1));
  std::string out;
  std::istringstream lines(text);
  for (std::string line; std::getline(lines, line); ) {
    if (!line.empty()) {
      out += add_tabs(depth) + line + "\n";
    }
  }
  return out;
}

bool HTNBTBuilder::holds_initially(const std::string & precondition) const
{
  auto predicates = initial_predicates();
  auto functions = initial_functions();
  return plansys2::check(parser::pddl::fromString(precondition), predicates, functions);
}

std::string HTNBTBuilder::task_xml(const std::string & id, int depth) const
{
  const plansys2_msgs::msg::PlanTask & task = *tasks_.at(id);
  const std::string name = task.task.empty() ? "initial task network" :
    task.task + " ← " + task.method;
  std::string out = add_tabs(depth) + "<Sequence name=\"" + escaped(name) + "\">\n";
  const auto ended = ended_by(task.applied, plan_.items.size());
  // On a replan, a method applied before anything of the new plan ends was applied in the
  // state the planner started from: the one before the actions that go on began, which have
  // changed it since. It is checked there, now, and not in the tree
  const bool checked = !task.precondition.empty() && ended.empty() && has_initial_state() &&
    holds_initially(task.precondition);
  if (!task.precondition.empty() && !checked) {
    // In the state the plan applied the method in: once what ended by then has ended
    out += waits_xml(ended, depth + 1);
    out += add_tabs(depth + 1) + "<CheckMethodPrecondition task=\"" + escaped(task.task) +
      "\" method=\"" + escaped(task.method) + "\" precondition=\"" +
      escaped(task.precondition) + "\"/>\n";
  }
  auto child = [&](const std::string & ref, int d) {
      return is_item(ref) ? action_xml(item_of(ref), d) : task_xml(ref, d);
    };
  const auto order = total_order(task.subtasks.size(), task.ordering);
  if (task.subtasks.empty()) {
    out += add_tabs(depth + 1) + "<AlwaysSuccess/>\n";
  } else if (order) {
    // One after another, as the method orders them
    for (size_t s : *order) {
      out += child(task.subtasks[s], depth + 1);
    }
  } else {
    // At once where the method does not order them: the actions wait for what the plan put
    // before them
    out += add_tabs(depth + 1) + "<Parallel success_count=\"" +
      std::to_string(task.subtasks.size()) + "\" failure_count=\"1\">\n";
    for (const auto & ref : task.subtasks) {
      out += child(ref, depth + 2);
    }
    out += add_tabs(depth + 1) + "</Parallel>\n";
  }
  out += add_tabs(depth) + "</Sequence>\n";
  return out;
}

std::string HTNBTBuilder::get_tree(const plansys2_msgs::msg::Plan & current_plan)
{
  plan_ = current_plan;
  tasks_.clear();
  if (plan_.tasks.empty()) {
    std::cerr << "HTNBTBuilder: the plan has no decomposition: it needs an HDDL planner" <<
      std::endl;
    return "";
  }
  for (const auto & task : plan_.tasks) {
    tasks_[task.id] = &task;
  }
  // Every subtask is a task of the plan or one of its actions
  for (const auto & task : plan_.tasks) {
    for (const auto & ref : task.subtasks) {
      if (is_item(ref) ? item_of(ref) >= plan_.items.size() : tasks_.count(ref) == 0) {
        std::cerr << "HTNBTBuilder: " << ref << " is not in the plan" << std::endl;
        return "";
      }
    }
  }
  // Each action waits for those that end before it starts
  waits_.assign(plan_.items.size(), {});
  for (size_t i = 0; i < plan_.items.size(); ++i) {
    waits_[i] = ended_by(plan_.items[i].time, i);
  }
  return std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n") +
         add_tabs(1) + "<BehaviorTree ID=\"MainTree\">\n" +
         task_xml(plan_.tasks.front().id, 2) +
         add_tabs(1) + "</BehaviorTree>\n</root>\n";
}

std::string HTNBTBuilder::get_dotgraph(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map, bool enable_legend,
  bool enable_print_graph)
{
  (void)enable_legend;
  // The decomposition: tasks as boxes with their method, actions coloured by their state
  std::stringstream ss;
  ss << "digraph plan {\n";
  for (size_t t = 0; t < plan_.tasks.size(); ++t) {
    const auto & task = plan_.tasks[t];
    ss << "\"" << task.id << "\" [shape=box,label=\"" <<
      (task.task.empty() ? std::string("initial task network") : task.task + "\\n" + task.method) <<
      "\"];\n";
    for (const auto & ref : task.subtasks) {
      ss << "\"" << task.id << "\" -> \"" << ref << "\";\n";
    }
  }
  for (size_t i = 0; i < plan_.items.size(); ++i) {
    const auto id = to_action_id(plan_.items[i], action_time_precision_);
    ss << "\"item:" << i << "\" [label=\"" << plan_.items[i].action << "\",style=filled";
    auto status = ActionExecutor::IDLE;
    if (action_map) {
      auto it = action_map->find(id);
      if (it != action_map->end() && it->second.action_executor) {
        status = it->second.action_executor->get_internal_status();
      }
    }
    switch (status) {
      case ActionExecutor::RUNNING: ss << ",color=blue,fillcolor=skyblue"; break;
      case ActionExecutor::SUCCESS: ss << ",color=green4,fillcolor=seagreen2"; break;
      case ActionExecutor::FAILURE:
      case ActionExecutor::CANCELLED: ss << ",color=red,fillcolor=pink"; break;
      default: ss << ",color=yellow3,fillcolor=lightgoldenrod1"; break;
    }
    ss << "];\n";
  }
  ss << "}\n";
  if (enable_print_graph) {
    std::cout << ss.str() << std::endl;
  }
  return ss.str();
}

}  // namespace plansys2
