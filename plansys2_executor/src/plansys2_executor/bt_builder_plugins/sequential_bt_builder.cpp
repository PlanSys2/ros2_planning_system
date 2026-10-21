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

#include "plansys2_executor/bt_builder_plugins/sequential_bt_builder.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace plansys2
{

SequentialBTBuilder::SequentialBTBuilder()
{
}

void SequentialBTBuilder::initialize(
  const std::string & bt_action_1, const std::string & bt_action_2, int precision)
{
  (void)bt_action_2;
  action_time_precision_ = precision;
  if (bt_action_1 != "") {
    bt_action_ = bt_action_1;
  } else {
    bt_action_ =
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
}

std::string SequentialBTBuilder::add_action_to_bt(
  const plansys2_msgs::msg::PlanItem & plan_item,
  const std::vector<plansys2_msgs::msg::PlanItem> & previous_items)
{
  std::string ret;
  std::string ret_aux = bt_action_;
  // The same ids as the executor gives to its actions
  const std::string action_id = to_action_id(plan_item, action_time_precision_);

  // The actions run one after the other: waiting for the previous one is enough
  std::string wait_actions;
  if (!previous_items.empty()) {
    wait_actions = add_tabs(1) + "<WaitAction action=\"" +
      to_action_id(previous_items.back(), action_time_precision_) + "\"/>\n";
  }

  replace(ret_aux, "ACTION_ID", action_id);
  replace(ret_aux, "WAIT_PREV_ACTIONS", wait_actions);

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {
    if (line != "") {
      ret = ret + add_tabs(3) + line + "\n";
    }
  }
  return ret;
}

std::string SequentialBTBuilder::get_tree(const plansys2_msgs::msg::Plan & current_plan)
{
  plan_ = current_plan;

  std::string bt_string;
  bt_string = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n") +
    add_tabs(1) + "<BehaviorTree ID=\"MainTree\">\n" + add_tabs(2) +
    "<Sequence name=\"MainSequence\">\n";

  std::vector<plansys2_msgs::msg::PlanItem> previous_items;
  for (const auto & item : current_plan.items) {
    std::string action_bt = add_action_to_bt(item, previous_items);
    bt_string += action_bt;
    previous_items.push_back(item);
  }

  bt_string += add_tabs(2) + "</Sequence>\n";
  bt_string += add_tabs(1) + "</BehaviorTree>\n</root>\n";

  return bt_string;
}

std::string SequentialBTBuilder::get_dotgraph(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map, bool enable_legend,
  bool enable_print_graph)
{
  (void)enable_legend;

  // The plan as a chain, each action coloured by its state
  std::stringstream ss;
  ss << "digraph plan {\n";
  for (size_t i = 0; i < plan_.items.size(); i++) {
    const auto id = to_action_id(plan_.items[i], action_time_precision_);
    ss << i << " [label=\"" << plan_.items[i].action << "\",labeljust=c,style=filled";
    auto status = ActionExecutor::IDLE;
    if (action_map) {
      auto it = action_map->find(id);
      if (it != action_map->end() && it->second.action_executor) {
        status = it->second.action_executor->get_internal_status();
      }
    }
    switch (status) {
      case ActionExecutor::RUNNING:
        ss << ",color=blue,fillcolor=skyblue";
        break;
      case ActionExecutor::SUCCESS:
        ss << ",color=green4,fillcolor=seagreen2";
        break;
      case ActionExecutor::FAILURE:
      case ActionExecutor::CANCELLED:
        ss << ",color=red,fillcolor=pink";
        break;
      default:
        ss << ",color=yellow3,fillcolor=lightgoldenrod1";
        break;
    }
    ss << "];\n";
    if (i > 0) {
      ss << i - 1 << "->" << i << ";\n";
    }
  }
  ss << "}\n";

  if (enable_print_graph) {
    std::cout << ss.str() << std::endl;
  }
  return ss.str();
}

}  // namespace plansys2
