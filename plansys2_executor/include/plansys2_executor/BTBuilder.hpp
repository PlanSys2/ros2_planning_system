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

#ifndef PLANSYS2_EXECUTOR__BTBUILDER_HPP_
#define PLANSYS2_EXECUTOR__BTBUILDER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <list>

#include "std_msgs/msg/empty.hpp"
#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_pddl_parser/Tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

struct RequirementConnection;
struct EffectConnection;


struct ActionUnit
{
  using Ptr = std::shared_ptr<ActionUnit>;
  static Ptr make_shared() {return std::make_shared<ActionUnit>();}

  std::string action;
  int time;

  std::list<std::shared_ptr<RequirementConnection>> reqs;
  std::list<std::shared_ptr<EffectConnection>> effects;
};

bool operator<(const ActionUnit::Ptr & op1, const ActionUnit::Ptr & op2);

struct RequirementConnection
{
  using Ptr = std::shared_ptr<RequirementConnection>;
  static Ptr make_shared() {return std::make_shared<RequirementConnection>();}

  std::shared_ptr<parser::pddl::tree::TreeNode> requirement;
  ActionUnit::Ptr action;
  bool satisfied;
  std::list<std::shared_ptr<EffectConnection>> effect_connections;
};

struct EffectConnection
{
  using Ptr = std::shared_ptr<EffectConnection>;
  static Ptr make_shared() {return std::make_shared<EffectConnection>();}

  std::shared_ptr<parser::pddl::tree::TreeNode> effect;
  std::shared_ptr<ActionUnit> action;
  std::list<RequirementConnection::Ptr> requirement_connections;
};

struct ExecutionLevel
{
  using Ptr = std::shared_ptr<ExecutionLevel>;
  static Ptr make_shared() {return std::make_shared<ExecutionLevel>();}

  int time;
  std::list<ActionUnit::Ptr> action_units;
};

class BTBuilder
{
public:
  explicit BTBuilder(rclcpp::Node::SharedPtr node);
  // void print(std::shared_ptr<GraphNode> current = root_) const;

  std::string get_tree(const Plan & current_plan);

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  std::vector<ExecutionLevel::Ptr> levels_;

  void print_levels(std::vector<ExecutionLevel::Ptr> & levels);
  bool level_satisfied(ExecutionLevel::Ptr level);
  void check_connections(ExecutionLevel::Ptr up_level, ExecutionLevel::Ptr down_level);

  std::string get_flow_tree(
    ActionUnit::Ptr root_flow, std::set<ActionUnit::Ptr> & used_actions, int level = 0);

  std::set<ActionUnit::Ptr> pred(ActionUnit::Ptr action_unit);
  std::set<ActionUnit::Ptr> succ(ActionUnit::Ptr action_unit);

  int in_cardinality(ActionUnit::Ptr action_unit);
  int out_cardinality(ActionUnit::Ptr action_unit);

  std::vector<ExecutionLevel::Ptr> get_plan_actions(const Plan & plan);
  std::string t(int level);

  std::string execution_block(const std::string & action, int plan_time, int l);
  // bool is_predecessor(const PlanItem & op1, const PlanItem & op2);
  // void add_child(GraphNode & parent, GraphNode & new_child);

  // std::shared_ptr<GraphNode> root_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BTBUILDER_HPP_
