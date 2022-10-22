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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__CONTINGENT_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__CONTINGENT_BT_BUILDER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <utility>
#include <tuple>

#include "std_msgs/msg/empty.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2 {
  struct GraphNode {
    using Ptr = std::shared_ptr<GraphNode>;

    static Ptr make_shared() { return std::make_shared<GraphNode>(); }

    ActionStamped action;
    int node_num;
    int level_num;

    std::vector<plansys2::Predicate> predicates;
    std::vector<plansys2::Function> functions;

    std::list<GraphNode::Ptr> in_arcs;
    std::list<GraphNode::Ptr> out_arcs;
  };

  struct Graph {
    using Ptr = std::shared_ptr<Graph>;

    static Ptr make_shared() { return std::make_shared<Graph>(); }

    std::list<GraphNode::Ptr> roots;
    std::map<float, std::list<GraphNode::Ptr>> levels;
  };


  class ContingentBTBuilder : public BTBuilder {
  public:
    ContingentBTBuilder();

    void initialize(
        const std::string &bt_action_1 = "",
        const std::string &bt_action_2 = "",
        int precision = 3);

    std::string get_tree(const plansys2_msgs::msg::Plan &current_plan);
    void get_sub_tree(const std::shared_ptr<plansys2::PlanNode> &root, const  std::string& indents, std::string& tree);
    void add_action_sequence(const plansys2_msgs::msg::PlanItem &item, const  std::string& indents, std::string& tree);
    void add_observe_action_sequence(bool observe_result, const plansys2_msgs::msg::PlanItem &item, const  std::string& indents, std::string& tree);

    std::string get_dotgraph(
        std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
        bool enable_legend = false,
        bool enable_print_graph = false);

  protected:
    std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

    Graph::Ptr graph_;
    std::string bt_;
    std::string bt_action_;

    std::string get_flow_tree(
        GraphNode::Ptr node,
        std::list<std::string> & used_nodes,
        int level = 0);
    void get_flow_dotgraph(GraphNode::Ptr node, std::set<std::string> & edges);
    std::string get_node_dotgraph(
        GraphNode::Ptr node, std::shared_ptr<std::map<std::string,
        ActionExecutionInfo>> action_map, int level = 0);

    ActionExecutor::Status get_action_status(
        ActionStamped action,
        std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

    void addDotGraphLegend(
        std::stringstream &ss, int tab_level, int level_counter,
        int node_counter);

    std::string t(int level);

  };

}  // namespace plansys2



#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
