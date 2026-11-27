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

#include "plansys2_executor/RemainingTasks.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace plansys2
{

namespace
{

// A subtask of the decomposition: a task of Plan.tasks or an item of the plan
struct Ref
{
  int task = -1;
  int item = -1;
};

class Remaining
{
public:
  Remaining(const plansys2_msgs::msg::Plan & plan, const std::vector<bool> & ended)
  : plan_(plan), ended_(ended)
  {
    for (size_t t = 0; t < plan.tasks.size(); ++t) {
      index_[plan.tasks[t].id] = static_cast<int>(t);
    }
  }

  std::string network()
  {
    // Path: the subtask taken at each task, from the root down
    std::vector<std::pair<int, size_t>> path;
    walk(0, path);
    std::string subtasks, ordering;
    for (size_t i = 0; i < kept_.size(); ++i) {
      subtasks += " (t" + std::to_string(i) + " " + kept_[i].text + ")";
      for (size_t j = 0; j < kept_.size(); ++j) {
        if (i != j && before(kept_[i].path, kept_[j].path)) {
          ordering += " (< t" + std::to_string(i) + " t" + std::to_string(j) + ")";
        }
      }
    }
    return "(:htn :subtasks (and" + subtasks + ")" +
           (ordering.empty() ? std::string() : " :ordering (and" + ordering + ")") + ")";
  }

private:
  struct Kept
  {
    std::string text;
    std::vector<std::pair<int, size_t>> path;
  };

  Ref ref(const std::string & subtask) const
  {
    Ref r;
    if (subtask.rfind("item:", 0) == 0) {
      r.item = std::stoi(subtask.substr(5));
    } else if (auto found = index_.find(subtask); found != index_.end()) {
      r.task = found->second;
    }
    return r;
  }

  // How many actions under a subtask, and how many of them ended
  std::pair<size_t, size_t> count(const Ref & r) const
  {
    if (r.item >= 0) {
      const bool done = static_cast<size_t>(r.item) < ended_.size() && ended_[r.item];
      return {1, done ? 1 : 0};
    }
    if (r.task < 0) {
      return {0, 0};
    }
    std::pair<size_t, size_t> total{0, 0};
    for (const auto & s : plan_.tasks[r.task].subtasks) {
      const auto [all, done] = count(ref(s));
      total.first += all;
      total.second += done;
    }
    return total;
  }

  // Whether subtask `a` of task `t` comes after one not done: an ordering of the method puts a
  // subtask with actions not all ended before it
  bool after_pending(int t, size_t a) const
  {
    const auto & task = plan_.tasks[t];
    for (size_t k = 0; k + 1 < task.ordering.size(); k += 2) {
      if (task.ordering[k + 1] == a && task.ordering[k] < task.subtasks.size()) {
        const auto [all, done] = count(ref(task.subtasks[task.ordering[k]]));
        if (done < all) {
          return true;
        }
      }
    }
    return false;
  }

  void walk(int t, std::vector<std::pair<int, size_t>> & path)
  {
    const auto & task = plan_.tasks[t];
    for (size_t s = 0; s < task.subtasks.size(); ++s) {
      const Ref r = ref(task.subtasks[s]);
      const auto [all, done] = count(r);
      path.emplace_back(t, s);
      if (r.item >= 0) {
        if (done == 0) {
          kept_.push_back({plan_.items[r.item].action, path});
        }
      } else if (r.task >= 0) {
        if (all == 0) {
          if (after_pending(t, s)) {
            kept_.push_back({plan_.tasks[r.task].task, path});
          }
        } else if (done == 0) {
          kept_.push_back({plan_.tasks[r.task].task, path});
        } else if (done < all) {
          walk(r.task, path);
        }
      }
      path.pop_back();
    }
  }

  // Whether the subtask at path `a` is ordered before the one at `b`: where the paths part, the
  // method of that task orders the first before the second, directly or through others
  bool before(
    const std::vector<std::pair<int, size_t>> & a,
    const std::vector<std::pair<int, size_t>> & b) const
  {
    size_t k = 0;
    while (k < a.size() && k < b.size() && a[k] == b[k]) {
      ++k;
    }
    if (k >= a.size() || k >= b.size() || a[k].first != b[k].first) {
      return false;
    }
    const auto & task = plan_.tasks[a[k].first];
    const size_t n = task.subtasks.size();
    // Reachability in the orderings of the method, from a's subtask to b's
    std::vector<bool> seen(n, false);
    std::vector<size_t> stack{a[k].second};
    while (!stack.empty()) {
      const size_t from = stack.back();
      stack.pop_back();
      for (size_t i = 0; i + 1 < task.ordering.size(); i += 2) {
        const size_t to = task.ordering[i + 1];
        if (task.ordering[i] == from && to < n && !seen[to]) {
          if (to == b[k].second) {
            return true;
          }
          seen[to] = true;
          stack.push_back(to);
        }
      }
    }
    return false;
  }

  const plansys2_msgs::msg::Plan & plan_;
  const std::vector<bool> & ended_;
  std::map<std::string, int> index_;
  std::vector<Kept> kept_;
};

}  // namespace

std::string remaining_task_network(
  const plansys2_msgs::msg::Plan & plan, const std::vector<bool> & ended)
{
  if (plan.tasks.empty()) {
    return "";
  }
  return Remaining(plan, ended).network();
}

std::string with_task_network(const std::string & problem, const std::string & network)
{
  const auto start = problem.find("(:htn");
  if (start == std::string::npos) {
    const auto init = problem.find("(:init");
    return init == std::string::npos ? problem :
           problem.substr(0, init) + network + "\n" + problem.substr(init);
  }
  int depth = 0;
  for (size_t i = start; i < problem.size(); ++i) {
    if (problem[i] == '(') {
      ++depth;
    } else if (problem[i] == ')' && --depth == 0) {
      return problem.substr(0, start) + network + problem.substr(i + 1);
    }
  }
  return problem;
}

}  // namespace plansys2
