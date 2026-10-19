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

#ifndef PLANSYS2_EXECUTOR__CONSTRAINTMONITOR_HPP_
#define PLANSYS2_EXECUTOR__CONSTRAINTMONITOR_HPP_

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

/**
 * @brief Follows the hard PDDL3 constraints of a domain and its problem while a plan runs.
 *
 * It is told each state (update) and the time (tick), in seconds since the plan started, and
 * says which constraint is violated as soon as it is; at the end of the plan (finish), those
 * that needed something that did not happen. Preferences are left out: they count only for
 * the planner. A forall is grounded over the objects of its types.
 */
class ConstraintMonitor
{
public:
  using Holds = std::function<bool(const plansys2_msgs::msg::Tree &)>;

  /// The constraints of a domain and its problem, as texts
  ConstraintMonitor(const std::string & domain, const std::string & problem);

  /// Constraints already grounded, as a tree of and of modal constraints
  explicit ConstraintMonitor(const plansys2_msgs::msg::Tree & constraints);

  /// Whether there is nothing to follow
  bool empty() const {return constraints_.empty();}

  /// Whether some constraint has times: the plan has to run at its times
  bool timed() const;

  /// The constraints followed, as text
  std::vector<std::string> describe() const;

  /// A new state at time t: the violated constraint, if one is
  std::optional<std::string> update(double t, const Holds & holds);

  /// Time passes, the state as it was: the violated constraint, if one is
  std::optional<std::string> tick(double t);

  /// The plan is over at time t, in its last state: the violated constraint, if one is
  std::optional<std::string> finish(double t);

private:
  struct Constraint
  {
    std::string op;
    std::vector<double> times;
    std::vector<plansys2_msgs::msg::Tree> conds;
    std::string text;
    // What the trajectory has given so far
    std::vector<bool> last;  // Truth of each condition in the last state
    bool seen = false;  // The (first) condition held in some state
    bool seen_second = false;  // The second one held in a state before this one
    int intervals = 0;  // How many times the condition became true
    std::optional<double> deadline;  // A pending need, by this time
  };

  void add(const plansys2_msgs::msg::Tree & tree, uint32_t id);
  std::optional<std::string> check_times(double t);

  std::vector<Constraint> constraints_;
  bool started_ = false;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__CONSTRAINTMONITOR_HPP_
