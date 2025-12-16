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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SEQUENTIAL_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SEQUENTIAL_BT_BUILDER_HPP_

#include <map>
#include <memory>
#include <vector>
#include <string>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2
{

class SequentialBTBuilder : public BTBuilder
{
public:
  SequentialBTBuilder();

  void initialize(
    const std::string & bt_action_1 = "", const std::string & bt_action_2 = "", int precision = 3);
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);
  plansys2::Graph::Ptr get_graph() {return nullptr;}
  bool propagate(plansys2::Graph::Ptr) {return true;}
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false, bool enable_print_graph = false);

  std::string add_action_to_bt(
    const plansys2_msgs::msg::PlanItem & plan_item,
    const std::vector<plansys2_msgs::msg::PlanItem> & previous_items);

protected:
  std::string bt_action_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
};

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::SequentialBTBuilder, plansys2::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SEQUENTIAL_BT_BUILDER_HPP_
