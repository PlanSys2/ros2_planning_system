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

#ifndef PLANSYS2_STATE_UPDATER__PROBLEM_WRITER_HPP_
#define PLANSYS2_STATE_UPDATER__PROBLEM_WRITER_HPP_

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/problem_change.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_problem_expert/ProblemUpdater.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "plansys2_state_updater/rules.hpp"

namespace plansys2_state_updater
{

/// Reads and writes the problem directly in the problem expert. The plan in course is asked to
/// the executor asynchronously: the problem expert never waits for it.
class ProblemWriter
{
public:
  explicit ProblemWriter(const plansys2::ProblemAccess & access);

  void start(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void stop();

  /// Atomically if possible, else one by one. Returns whether each change was made
  std::vector<bool> write(const std::vector<Change> & changes);

  std::set<std::string> predicates() const;
  std::map<std::string, double> functions() const;
  /// Object fluents: (pos r1) -> kitchen
  std::map<std::string, std::string> objects() const;
  std::set<std::string> instances() const;
  std::optional<double> current_value(const std::string & fact) const;

  /// Whether the goal or the rest of the last known plan uses an object
  bool in_use(const std::string & name);

  /// For tests
  void set_plan(const plansys2_msgs::msg::Plan & plan) {plan_ = plan;}

  static plansys2_msgs::msg::ProblemChange problem_change(const Change & change);

private:
  void ask_plan();

  plansys2::ProblemAccess access_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr plan_client_;
  plansys2_msgs::msg::Plan plan_;
  double last_ask_ {-1e9};
};

}  // namespace plansys2_state_updater

#endif  // PLANSYS2_STATE_UPDATER__PROBLEM_WRITER_HPP_
