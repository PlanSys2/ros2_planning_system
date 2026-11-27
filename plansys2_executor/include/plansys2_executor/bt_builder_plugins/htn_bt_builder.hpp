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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__HTN_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__HTN_BT_BUILDER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2
{

/**
 * @class plansys2::HTNBTBuilder
 * @brief The behavior tree of a hierarchical (HTN) plan, as HDDL executes it.
 *
 * The tree is the decomposition of the plan (Plan.tasks), not a graph of its actions:
 * - every task is a node, named after the task and its method, that succeeds when all of its
 *   subtasks have, and fails, and fails those above it, when one of them fails;
 * - the precondition of its method is checked (CheckMethodPrecondition) when the plan applied
 *   it (PlanTask.applied: once the actions that end by then have ended), and before its
 *   subtasks run: if the world is no longer as when the plan was made, the task fails there,
 *   before any of its actions start;
 * - the subtasks of a method run in the order the method gives them: one after another where
 *   it orders them all, at once where it does not;
 * - the actions are the leaves (the action template of the executor), each waiting for those
 *   that end before it starts in the plan: what the order of the methods leaves open is done as
 *   the planner found it, at once where the plan has times that overlap.
 *
 * It needs a plan with its decomposition (an HDDL planner, such as Klimax), and the executor
 * uses it in HDDL mode only.
 */
class HTNBTBuilder : public BTBuilder
{
public:
  HTNBTBuilder() = default;

  void initialize(
    const std::string & bt_action_1 = "", const std::string & bt_action_2 = "",
    int precision = 3) override;
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan) override;
  plansys2::Graph::Ptr get_graph() override {return nullptr;}
  bool propagate(plansys2::Graph::Ptr) override {return true;}
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false, bool enable_print_graph = false) override;

  /// @brief Whether the subtasks of a method are totally ordered by `ordering` (pairs, one
  /// after the other), and in which order.
  static std::optional<std::vector<size_t>> total_order(
    size_t subtasks, const std::vector<uint32_t> & ordering);

protected:
  // Whether a precondition holds in the state the plan starts from (initial_state)
  bool holds_initially(const std::string & precondition) const;
  std::string task_xml(const std::string & id, int depth) const;
  std::string action_xml(size_t item, int depth) const;
  std::string waits_xml(const std::vector<size_t> & items, int depth) const;
  // The actions that end by `time`, but for those that end before another of them starts
  // (that one waits for them already); `item` itself left out
  std::vector<size_t> ended_by(double time, size_t item) const;

  std::string bt_action_;
  plansys2_msgs::msg::Plan plan_;
  std::map<std::string, const plansys2_msgs::msg::PlanTask *> tasks_;
  std::vector<std::vector<size_t>> waits_;  // per item: the items it waits for
};

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::HTNBTBuilder, plansys2::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__HTN_BT_BUILDER_HPP_
