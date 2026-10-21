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

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_core/Action.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

struct DerivedRule;  // plansys2_problem_expert/Utils.hpp

class DomainExpertClient;
class ProblemExpertClient;

/**
 * @brief Enumeration of action types in a plan.
 *
 * Defines the different types of actions that can appear in a planning system,
 * including durative actions and their temporal phases.
 */
enum struct ActionType
{
  UNKNOWN,
  INIT,
  DURATIVE,
  START,
  OVERALL,
  END,
  GOAL,
  TIMED  // A timed literal or fluent: only effects, at its time; the problem expert applies them
};

/**
 * @brief Representation of an action with its temporal information.
 *
 * Contains all necessary information about an action in a plan, including
 * when it occurs, how long it lasts, and what type of action it is.
 */
struct ActionStamped
{
  double time;
  std::string expression;
  double duration;
  ActionType type {};
  ActionVariant action;

  ActionStamped()
  : time(0.0), duration(0.0) {}
};

/**
 * @brief Node in the temporal planning graph.
 *
 * Represents an action in the temporal planning graph with its input and output
 * dependencies. Used to construct the execution flow of the behavior tree.
 */
struct Node
{
  using Ptr = std::shared_ptr<Node>;
  static Ptr make_shared(int id) {return std::make_shared<Node>(id);}

  int node_num;
  ActionStamped action;

  std::set<std::tuple<Node::Ptr, double, double>> input_arcs;
  std::set<std::tuple<Node::Ptr, double, double>> output_arcs;

  explicit Node(int id)
  : node_num(id) {}
};

/**
 * @brief Temporal planning graph representation.
 *
 * Contains a collection of nodes that represent the actions in a plan
 * and their temporal relationships.
 */
struct Graph
{
  using Ptr = std::shared_ptr<Graph>;
  static Ptr make_shared() {return std::make_shared<Graph>();}

  std::list<Node::Ptr> nodes;
  // Decimals of the action ids of its nodes, those of the action map
  int action_time_precision = 3;

  // Arcs point both ways, so the nodes would keep each other alive (#430)
  ~Graph()
  {
    for (const auto & node : nodes) {
      node->input_arcs.clear();
      node->output_arcs.clear();
    }
  }
};

inline std::string add_tabs(int level)
{
  return std::string(level * 2, ' ');
}

/**
 * @brief Replaces all occurrences of a substring in a string.
 *
 * @param[in,out] str The string to modify.
 * @param[in] from The substring to replace.
 * @param[in] to The replacement substring.
 */
inline void replace(std::string & str, const std::string & from, const std::string & to)
{
  if (from.empty()) {return;}
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

/**
 * @class plansys2::BTBuilder
 * @brief Interface for behavior tree builder implementations.
 *
 * This abstract class defines the interface for components that transform
 * a plan into a behavior tree representation for execution.
 */
class BTBuilder
{
public:
  using Ptr = std::shared_ptr<plansys2::BTBuilder>;

  // Plugins are destroyed through this class; without it their clients leaked (#430)
  virtual ~BTBuilder() = default;

  /**
   * @brief Set the clients the builder uses: those of its executor, on the executor's node.
   *
   * Required before building a tree: a builder makes no node of its own.
   *
   * @param[in] domain_client The domain expert client.
   * @param[in] problem_client The problem expert client.
   */
  void set_clients(
    std::shared_ptr<plansys2::DomainExpertClient> domain_client,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client);

  /**
   * @brief Initialize the builder with behavior tree templates.
   *
   * @param[in] bt_action_1 XML template for regular actions, default empty.
   * @param[in] bt_action_2 XML template for durative actions, default empty.
   * @param[in] precision Precision for time calculations, default 3.
   */
  virtual void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3) = 0;

  /**
   * @brief Generate a behavior tree XML from a plan.
   *
   * @param[in] current_plan The plan to transform into a behavior tree.
   * @return std::string containing the behavior tree XML.
   */
  virtual std::string get_tree(const plansys2_msgs::msg::Plan & current_plan) = 0;

  /**
   * @brief Get the internal graph representation.
   *
   * @return Shared pointer to the graph.
   */
  virtual Graph::Ptr get_graph() = 0;

  /**
   * @brief Propagate temporal constraints through the graph.
   *
   * @param[in,out] graph The graph to propagate constraints through.
   * @return True if propagation was successful, false if inconsistencies were found.
   */
  virtual bool propagate(Graph::Ptr graph) = 0;

  /**
   * @brief Generate a DOT graph representation for visualization.
   *
   * @param[in] action_map Map of action IDs to execution information.
   * @param[in] enable_legend Whether to include a legend in the graph, default false.
   * @param[in] enable_print_graph Whether to print the graph to console, default false.
   * @return std::string containing the DOT graph representation.
   */
  virtual std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false,
    bool enable_print_graph = false) = 0;

  /**
   * @brief Convert a floating point time to an integer with specified precision.
   *
   * @param[in] time The time value to convert.
   * @param[in] power The power of 10 to multiply by (precision).
   * @return The time as an integer with the specified precision.
   */
  static int64_t to_int_time(double time, int power)
  {
    // Rounded: truncating made 4.9999 and 4.9990 the same time, and 5.0005 earlier than 5.0004
    return std::llround(time * std::pow(10.0, power));
  }

  /**
   * @brief Convert an action type to its string representation.
   *
   * @param[in] action_type The action type to convert.
   * @return std::string representation of the action type.
   */
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
      case ActionType::TIMED:
        return "TIMED";
      default:
        return "UNKNOWN";
    }
  }

  /**
   * @brief Generate a unique action identifier from a plan item.
   *
   * @param[in] item The plan item to generate an ID for.
   * @param[in] precision The precision for the time component.
   * @return std::string The unique identifier for the action.
   */
  static std::string to_action_id(const plansys2_msgs::msg::PlanItem & item, int precision)
  {
    return item.action + ":" + std::to_string(to_int_time(item.time, precision));
  }

  /**
   * @brief Generate a unique action identifier from an ActionStamped.
   *
   * @param[in] action The action to generate an ID for.
   * @param[in] precision The precision for the time component.
   * @return std::string The unique identifier for the action.
   */
  static std::string to_action_id(const ActionStamped & action, int precision)
  {
    return action.expression + ":" + std::to_string(to_int_time(action.time, precision));
  }

  /**
   * @brief The action of a plan item, from the domain, with ?duration bound to its duration.
   *
   * @param[in] domain_client Where the actions come from.
   * @param[in] actions The non-durative actions of the domain.
   * @param[in] item The plan item.
   * @return The action; empty if the domain does not have it.
   */
  static plansys2::ActionVariant plan_item_action(
    plansys2::DomainExpertClient & domain_client, const std::vector<std::string> & actions,
    const plansys2_msgs::msg::PlanItem & item);

  /**
   * @brief Set the state the plan starts from, instead of the current one.
   *
   * On a replan, the actions that go on started before the new plan: it starts from the state
   * before their at start effects.
   *
   * @param[in] predicates The predicates of that state.
   * @param[in] functions The functions of that state.
   */
  void set_initial_state(
    std::vector<plansys2::Predicate> predicates, std::vector<plansys2::Function> functions);

  /**
   * @brief How long ago the problem of the plan was read: its timed facts, written from now,
   *        are this much later in the time of the plan.
   */
  void set_time_offset(double seconds) {time_offset_ = seconds;}

  /**
   * @brief Decimals of the times in the action ids, as given to initialize.
   * @return The decimals.
   */
  int get_action_time_precision() const {return action_time_precision_;}

protected:
  // Decimals of the action ids; the executor gives its action_time_precision to initialize
  int action_time_precision_ = 3;

  // The clients set, or our own ones, created the first time they are needed
  std::shared_ptr<plansys2::DomainExpertClient> domain_client() const;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client() const;

  // The state the plan starts from: the one set, or that of the problem expert now
  std::vector<plansys2::Predicate> initial_predicates() const;
  std::vector<plansys2::Function> initial_functions() const;

  // Applies effects to a simulated state, whose derived predicates follow
  void apply_effects(
    const plansys2_msgs::msg::Tree & effects, std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;
  // Forgets the derived predicate rules, the timed facts and the dynamic facts: a new plan reads
  // them again
  void reset_derived_rules() const
  {
    derived_rules_.reset();
    derived_names_.reset();
    current_state_.reset();
    timed_events_.reset();
    dynamic_names_.reset();
    dynamic_functions_.reset();
  }

  // PDDL+: whether conditions read a fact the dynamics change (processes, events, continuous
  // effects). A discrete simulation of the plan cannot tell them: its tree waits for them
  bool reads_dynamic(const plansys2_msgs::msg::Tree & conditions) const;

  // The derived predicates of the domain, by name
  const std::set<std::string> & derived_names() const;

  // PDDL+: the conditions with the comparisons on what the dynamics change taken as held
  plansys2_msgs::msg::Tree relax_dynamic(const plansys2_msgs::msg::Tree & conditions) const;

  // The timed facts of the problem, as events of the plan at their time, with only effects
  std::vector<ActionStamped> timed_events() const;

private:
  void collect_dynamic() const;
  void load_derived() const;

  std::optional<std::pair<std::vector<plansys2::Predicate>, std::vector<plansys2::Function>>>
  initial_state_;

  // Mutable: creating them when first used does not change what the builder is
  mutable std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  mutable std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  mutable std::shared_ptr<const std::vector<plansys2::DerivedRule>> derived_rules_;
  mutable std::optional<std::set<std::string>> derived_names_;
  mutable std::optional<std::pair<std::vector<plansys2::Predicate>,
    std::vector<plansys2::Function>>> current_state_;
  mutable std::optional<std::vector<ActionStamped>> timed_events_;
  double time_offset_ {0.0};
  mutable std::optional<std::set<std::string>> dynamic_names_;
  mutable std::optional<std::set<std::string>> dynamic_functions_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BTBUILDER_HPP_
