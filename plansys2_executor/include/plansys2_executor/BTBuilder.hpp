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

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{
namespace bt_builder
{

enum struct ActionType { UNKNOWN, INIT, DURATIVE, START, OVERALL, END, GOAL };

struct ActionStamped
{
  float time;
  std::string expression;
  float duration;
  ActionType type;
  ActionVariant action;

  ActionStamped()
  : time(0.0), duration(0.0) {}
};

struct Node
{
  using Ptr = std::shared_ptr<Node>;
  static Ptr make_shared(int id) {return std::make_shared<Node>(id);}

  int node_num;
  ActionStamped action;

  std::set<std::tuple<plansys2::bt_builder::Node::Ptr, double, double>> input_arcs;
  std::set<std::tuple<plansys2::bt_builder::Node::Ptr, double, double>> output_arcs;

  explicit Node(int id)
  : node_num(id) {}
};

struct Graph
{
  using Ptr = std::shared_ptr<plansys2::bt_builder::Graph>;
  static Ptr make_shared() {return std::make_shared<plansys2::bt_builder::Graph>();}

  std::list<plansys2::bt_builder::Node::Ptr> nodes;
};

inline std::string add_tabs(int level)
{
  return std::string(level * 2, ' ');
}

inline void replace(std::string & str, const std::string & from, const std::string & to)
{
  if (from.empty()) return;
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

class BTBuilder
{
public:
  using Ptr = std::shared_ptr<plansys2::bt_builder::BTBuilder>;

  virtual void initialize(
    const std::string & bt_action_1 = "", const std::string & bt_action_2 = "",
    int precision = 3) = 0;

  virtual std::string get_tree(const plansys2_msgs::msg::Plan & current_plan) = 0;
  virtual plansys2::bt_builder::Graph::Ptr get_graph() = 0;
  virtual bool propagate(plansys2::bt_builder::Graph::Ptr graph) = 0;
  virtual std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false, bool enable_print_graph = false) = 0;

  static int to_int_time(float time, int power)
  {
    float scale = pow(10.0, static_cast<float>(power));
    return static_cast<int>(time * scale);
  }

  static std::string to_string(const ActionType & action_type)
  {
    switch (action_type) {
      case ActionType::INIT:
        return "INIT";
      case ActionType::DURATIVE:
        return "DURATIVE";
      case ActionType::START:
        return "START";
      case ActionType::OVERALL:
        return "OVERALL";
      case ActionType::END:
        return "END";
      case ActionType::GOAL:
        return "GOAL";
      default:
        return "UNKNOWN";
    }
  }

  static std::string to_action_id(const plansys2_msgs::msg::PlanItem & item, int precision)
  {
    return item.action + ":" + std::to_string(to_int_time(item.time, precision));
  }

  static std::string to_action_id(const ActionStamped & action, int precision)
  {
    return action.expression + ":" + std::to_string(to_int_time(action.time, precision));
  }

  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan)
  {
    std::vector<ActionStamped> ret;

    for (auto & item : plan.items) {
      ActionStamped action_stamped;

      action_stamped.time = item.time;
      action_stamped.duration = item.duration;
      auto actions = domain_client_->getActions();
      if (std::find(actions.begin(), actions.end(), get_action_name(item.action)) != actions.end()) {
        action_stamped.action =
          domain_client_->getAction(get_action_name(item.action), get_action_params(item.action));
      } else {
        action_stamped.action = domain_client_->getDurativeAction(
          get_action_name(item.action), get_action_params(item.action));
      }

      ret.push_back(action_stamped);
    }

    return ret;
  }

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
};

}  // namespace bt_builder
}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BTBUILDER_HPP_
