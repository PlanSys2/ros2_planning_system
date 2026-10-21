// Copyright 2019 Intelligent Robotics Lab
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

#include <algorithm>
#include <functional>
#include <cmath>
#include <tuple>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <utility>

#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

namespace
{

// Where evaluate reads, and applies, the facts that are not in a local state
class FactSource
{
public:
  virtual ~FactSource() = default;
  // std::nullopt when there is no answer
  virtual std::optional<bool> exist(const plansys2_msgs::msg::Node & predicate) = 0;
  virtual std::optional<plansys2_msgs::msg::Node> function(const std::string & expression) = 0;
  virtual std::optional<std::vector<plansys2::Instance>> instances() = 0;
  // All the facts that hold, derived ones included
  virtual std::optional<std::vector<plansys2_msgs::msg::Node>> facts() = 0;
  virtual bool add(const plansys2_msgs::msg::Node & predicate) = 0;
  virtual bool remove(const plansys2_msgs::msg::Node & predicate) = 0;
  virtual bool update(const plansys2::Function & function) = 0;
  virtual bool add_function(const plansys2::Function & function) = 0;
};

// The facts of a problem expert, through its services
class ClientSource : public FactSource
{
public:
  explicit ClientSource(std::shared_ptr<plansys2::ProblemExpertClient> client)
  : client_(client) {}

  std::optional<bool> exist(const plansys2_msgs::msg::Node & predicate) override
  {
    return client_->tryExistPredicate(predicate);
  }
  std::optional<plansys2_msgs::msg::Node> function(const std::string & expression) override
  {
    return client_->getFunction(expression);
  }
  std::optional<std::vector<plansys2::Instance>> instances() override
  {
    return client_->tryGetInstances();
  }
  std::optional<std::vector<plansys2_msgs::msg::Node>> facts() override
  {
    auto predicates = client_->tryGetPredicates();
    if (!predicates.has_value()) {
      return {};
    }
    return std::vector<plansys2_msgs::msg::Node>(predicates->begin(), predicates->end());
  }
  bool add(const plansys2_msgs::msg::Node & predicate) override
  {
    return client_->addPredicate(predicate);
  }
  bool remove(const plansys2_msgs::msg::Node & predicate) override
  {
    return client_->removePredicate(predicate);
  }
  bool update(const plansys2::Function & function) override
  {
    return client_->updateFunction(function);
  }
  bool add_function(const plansys2::Function & function) override
  {
    return client_->addFunction(function);
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> client_;
};

// The facts of the problem expert itself: what its clients would get, derived predicates and
// objects without facts included
class ExpertSource : public FactSource
{
public:
  explicit ExpertSource(plansys2::ProblemExpert & expert)
  : expert_(expert) {}

  std::optional<bool> exist(const plansys2_msgs::msg::Node & predicate) override
  {
    return expert_.existPredicate(predicate);
  }
  std::optional<plansys2_msgs::msg::Node> function(const std::string & expression) override
  {
    return expert_.getFunction(expression);
  }
  std::optional<std::vector<plansys2::Instance>> instances() override
  {
    return expert_.getInstances();
  }
  std::optional<std::vector<plansys2_msgs::msg::Node>> facts() override
  {
    auto predicates = expert_.getPredicates();
    return std::vector<plansys2_msgs::msg::Node>(predicates.begin(), predicates.end());
  }
  bool add(const plansys2_msgs::msg::Node & predicate) override
  {
    return expert_.addPredicate(predicate);
  }
  bool remove(const plansys2_msgs::msg::Node & predicate) override
  {
    return expert_.removePredicate(predicate);
  }
  bool update(const plansys2::Function & function) override
  {
    return expert_.updateFunction(function);
  }
  bool add_function(const plansys2::Function & function) override
  {
    return expert_.addFunction(function);
  }

private:
  plansys2::ProblemExpert & expert_;
};

// Where each variable of a quantifier appears in its body, as (node, parameter): a quantifier
// inside that declares the same name hides it
void collectPlaces(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id,
  const std::vector<plansys2_msgs::msg::Param> & variables,
  std::vector<std::vector<std::pair<uint32_t, size_t>>> & places,
  std::vector<bool> hidden)
{
  const auto & node = tree.nodes[node_id];
  const bool quantifier = node.node_type == plansys2_msgs::msg::Node::EXISTS ||
    node.node_type == plansys2_msgs::msg::Node::FORALL;
  for (size_t p = 0; p < node.parameters.size(); p++) {
    for (size_t v = 0; v < variables.size(); v++) {
      if (node.parameters[p].name != variables[v].name || hidden[v]) {continue;}
      if (quantifier) {
        hidden[v] = true;  // Declared again: not ours below
      } else {
        places[v].emplace_back(node_id, p);
      }
    }
  }
  for (auto child : node.children) {
    collectPlaces(tree, child, variables, places, hidden);
  }
}

// Calls visit with each combination of candidates, until it returns true
bool forEachTuple(
  const std::vector<std::vector<std::string>> & candidates,
  const std::function<bool(const std::vector<std::string> &)> & visit)
{
  for (const auto & c : candidates) {
    if (c.empty()) {return false;}
  }
  std::vector<size_t> index(candidates.size(), 0);
  std::vector<std::string> values(candidates.size());
  for (size_t i = 0; i < candidates.size(); i++) {
    values[i] = candidates[i][0];
  }
  while (true) {
    if (visit(values)) {return true;}
    size_t i = 0;
    while (i < candidates.size()) {
      if (++index[i] < candidates[i].size()) {
        values[i] = candidates[i][index[i]];
        break;
      }
      index[i] = 0;
      values[i] = candidates[i][0];
      i++;
    }
    if (i == candidates.size()) {return false;}
  }
}

// The effects of an action happen at once (PDDL): numeric values are computed from the state
// before it, deletes go before adds (adding a fact wins over deleting it), and the values are
// written at the end. ALL applies each effect as it is found
enum class ApplyPhase {ALL, NUMERIC, DELETES, ADDS};

// An object fluent set by an effect, read in the NUMERIC phase and written at the end
struct ObjectUpdate
{
  plansys2_msgs::msg::Node function;
  std::string before;  // Empty if it had none
  std::string object;
};

// A numeric effect, computed in the NUMERIC phase and written at the end
struct NumericUpdate
{
  plansys2_msgs::msg::Node function;
  uint8_t modifier_type = 0;
  double before = 0.0;  // The value before the action
  double value = 0.0;  // assign and scale: the new value
  double delta = 0.0;  // increase and decrease: what is added
};

// What the NUMERIC phase finds: the numeric effects, and the facts added (not deleted then)
struct ApplyPlan
{
  std::vector<NumericUpdate> numeric;
  std::vector<plansys2_msgs::msg::Node> adds;
  std::vector<ObjectUpdate> objects;
  // Conditional effects: whether each condition held before the action, by its text
  std::map<std::string, bool> conditions;
};

// The object a node stands for: a constant, a bound parameter, or the object an object fluent
// has (std::nullopt if none, or if it is a number)
std::optional<std::string> object_of(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, FactSource * source,
  const std::vector<plansys2::Function> & functions, bool use_state)
{
  const auto & node = tree.nodes[node_id];
  if (node.node_type == plansys2_msgs::msg::Node::CONSTANT) {
    return node.name;
  }
  if (node.node_type == plansys2_msgs::msg::Node::PARAMETER) {
    if (!node.parameters.empty() && node.parameters[0].name.front() != '?') {
      return node.parameters[0].name;
    }
    return {};
  }
  if (node.node_type != plansys2_msgs::msg::Node::FUNCTION) {
    return {};
  }
  std::optional<plansys2_msgs::msg::Node> function;
  if (use_state) {
    auto it = std::find_if(
      functions.begin(), functions.end(),
      std::bind(&parser::pddl::checkNodeEquality, std::placeholders::_1, node, true));
    if (it != functions.end()) {
      function = *it;
    }
  } else if (source) {
    function = source->function(parser::pddl::toString(tree, node_id));
  }
  if (!function || function->object.empty()) {
    return {};
  }
  return function->object;
}

bool is_object_node(const plansys2_msgs::msg::Node & node)
{
  return node.node_type == plansys2_msgs::msg::Node::CONSTANT ||
         node.node_type == plansys2_msgs::msg::Node::PARAMETER;
}

std::tuple<bool, bool, double> evaluate_node(
  const plansys2_msgs::msg::Tree & tree,
  FactSource * source,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  bool use_state,
  uint32_t node_id,
  bool negate,
  std::vector<FactChange> * changes,
  ApplyPhase phase = ApplyPhase::ALL,
  ApplyPlan * plan = nullptr)
{
  if (tree.nodes.empty()) {  // No expression
    return std::make_tuple(true, true, 0);
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND:
    case plansys2_msgs::msg::Node::OR: {
        // Children come back already negated: by De Morgan, (not (and a b)) is true if any
        // negated child is, and (not (or a b)) if all of them are
        const bool all = (tree.nodes[node_id].node_type == plansys2_msgs::msg::Node::AND) != negate;
        bool success = true;
        bool truth_value = all;

        for (auto & child_id : tree.nodes[node_id].children) {
          std::tuple<bool, bool, double> result =
            evaluate_node(
            tree, source, predicates, functions, apply, use_state, child_id,
            negate, changes, phase, plan);
          success = success && std::get<0>(result);
          truth_value = all ? (truth_value && std::get<1>(result)) :
            (truth_value || std::get<1>(result));
        }
        return std::make_tuple(success, truth_value, 0);
      }

    case plansys2_msgs::msg::Node::NOT: {
        return evaluate_node(
          tree, source, predicates, functions, apply, use_state,
          tree.nodes[node_id].children[0],
          !negate, changes, phase, plan);
      }

    case plansys2_msgs::msg::Node::WHEN: {
        // As a condition it asks for nothing
        if (!apply) {
          return std::make_tuple(true, true, 0);
        }
        // Its condition is read before the action changes anything: in the first phase, kept
        // for the others by its text, bound by any forall around
        const auto condition_id = tree.nodes[node_id].children[0];
        const auto key = std::to_string(node_id) + parser::pddl::toString(tree, condition_id);
        bool holds = false;
        if (plan && phase != ApplyPhase::NUMERIC && plan->conditions.count(key)) {
          holds = plan->conditions.at(key);
        } else {
          holds = std::get<1>(
            evaluate_node(
              tree, source, predicates, functions, false, use_state, condition_id, false,
              nullptr));
          if (plan) {
            plan->conditions[key] = holds;
          }
        }
        if (!holds) {
          return std::make_tuple(true, true, 0);
        }
        return evaluate_node(
          tree, source, predicates, functions, true, use_state, tree.nodes[node_id].children[1],
          negate, changes, phase, plan);
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        bool success = true;
        bool value = true;

        if (apply) {
          if (phase == ApplyPhase::NUMERIC && plan && !negate) {
            plan->adds.push_back(tree.nodes[node_id]);
          }
          const bool also_added = phase == ApplyPhase::DELETES && negate && plan &&
            std::any_of(
            plan->adds.begin(), plan->adds.end(), [&](const plansys2_msgs::msg::Node & n) {
              return parser::pddl::checkNodeEquality(n, tree.nodes[node_id], true);
            });
          if ((phase == ApplyPhase::NUMERIC) || (phase == ApplyPhase::DELETES && !negate) ||
            (phase == ApplyPhase::ADDS && negate) || also_added)
          {
            return std::make_tuple(true, !negate, 0);  // Applied in another phase
          }
          if (use_state) {
            auto it =
              std::find_if(
              predicates.begin(), predicates.end(),
              std::bind(
                &parser::pddl::checkNodeEquality, std::placeholders::_1,
                tree.nodes[node_id], true));
            if (negate) {
              if (it != predicates.end()) {
                predicates.erase(it);
              }
              value = false;
            } else {
              if (it == predicates.end()) {
                predicates.push_back(tree.nodes[node_id]);
              }
            }
          } else {
            // Only real changes are recorded: a fact that already held is left alone on undo
            std::optional<bool> existed;
            if (changes) {
              existed = source->exist(tree.nodes[node_id]);
              if (!existed.has_value()) {
                return std::make_tuple(false, false, 0);
              }
            }
            if (negate) {
              success = success && source->remove(tree.nodes[node_id]);
              value = false;
            } else {
              success = success && source->add(tree.nodes[node_id]);
            }
            if (changes && success && existed.value() == negate) {
              FactChange change;
              change.fact = tree.nodes[node_id];
              change.existed_before = existed.value();
              changes->push_back(change);
            }
          }
        } else {
          // negate | exist | output
          //   F    |   F   |   F
          //   F    |   T   |   T
          //   T    |   F   |   T
          //   T    |   T   |   F
          if (use_state) {
            value = negate ^
              (std::find_if(
                predicates.begin(), predicates.end(),
                std::bind(
                  &parser::pddl::checkNodeEquality, std::placeholders::_1,
                  tree.nodes[node_id], true)) != predicates.end());
          } else {
            auto exist = source->exist(tree.nodes[node_id]);
            // Without an answer it cannot be taken as true, neither negated
            if (!exist.has_value()) {
              return std::make_tuple(false, false, 0);
            }
            value = negate ^ exist.value();
          }
        }

        return std::make_tuple(success, value, 0);
      }

    case plansys2_msgs::msg::Node::FUNCTION: {
        bool success = true;
        double value = 0;

        if (use_state) {
          auto it =
            std::find_if(
            functions.begin(), functions.end(),
            std::bind(
              &parser::pddl::checkNodeEquality, std::placeholders::_1,
              tree.nodes[node_id], true));
          if (it != functions.end()) {
            value = it->value;
          } else {
            success = false;
          }
        } else {
          std::optional<plansys2_msgs::msg::Node> func =
            source->function(parser::pddl::toString(tree, node_id));

          if (func.has_value()) {
            value = func.value().value;
          } else {
            success = false;
          }
        }

        return std::make_tuple(success, false, value);
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        // Objects: an object fluent, a constant or a parameter, compared by name. A fluent with
        // no object yet is equal to none
        if (tree.nodes[node_id].expression_type == plansys2_msgs::msg::Node::COMP_EQ) {
          const auto first = tree.nodes[node_id].children[0];
          const auto second = tree.nodes[node_id].children[1];
          const auto first_object = object_of(tree, first, source, functions, use_state);
          const auto second_object = object_of(tree, second, source, functions, use_state);
          if (first_object || second_object || is_object_node(tree.nodes[first]) ||
            is_object_node(tree.nodes[second]))
          {
            const bool equal = first_object && second_object && *first_object == *second_object;
            return std::make_tuple(true, negate ^ equal, 0);
          }
        }

        // Numbers are not negated: only the comparison is
        std::tuple<bool, bool, double> left = evaluate_node(
          tree, source, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[0], false, changes, phase,
            plan);
        std::tuple<bool, bool, double> right = evaluate_node(
          tree, source, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[1], false, changes, phase,
            plan);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          return std::make_tuple(false, false, 0);
        }

        switch (tree.nodes[node_id].expression_type) {
          case plansys2_msgs::msg::Node::COMP_GE:
            if (std::get<2>(left) >= std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_GT:
            if (std::get<2>(left) > std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LE:
            if (std::get<2>(left) <= std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LT:
            if (std::get<2>(left) < std::get<2>(right)) {
              return std::make_tuple(true, negate ^ true, 0);
            } else {
              return std::make_tuple(true, negate ^ false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::COMP_EQ: {
              auto c_t = plansys2_msgs::msg::Node::CONSTANT;
              auto p_t = plansys2_msgs::msg::Node::PARAMETER;
              auto c0 = tree.nodes[tree.nodes[node_id].children[0]];
              auto c1 = tree.nodes[tree.nodes[node_id].children[1]];
              auto c0_type = c0.node_type;
              auto c1_type = c1.node_type;
              if ((c0_type == c_t || c0_type == p_t) && (c1_type == c_t || c1_type == p_t)) {
                std::string c0_name = (c0_type == p_t) ? c0.parameters[0].name : c0.name;
                std::string c1_name = (c1_type == p_t) ? c1.parameters[0].name : c1.name;
                return std::make_tuple(
                  true,
                  negate ^ (c1_name == c0_name),
                  0);
              }
              if (c0_type == c_t || c0_type == p_t || c1_type == c_t || c1_type == p_t) {
                break;  // an object is not equal to a number
              }
              // Numbers, functions and arithmetic: equal up to rounding
              const double l = std::get<2>(left);
              const double r = std::get<2>(right);
              const bool equal =
                std::abs(l - r) <= 1e-9 * std::max({1.0, std::abs(l), std::abs(r)});
              return std::make_tuple(true, negate ^ equal, 0);
            }
          case plansys2_msgs::msg::Node::ARITH_MULT:
            return std::make_tuple(true, false, std::get<2>(left) * std::get<2>(right));
            break;
          case plansys2_msgs::msg::Node::ARITH_DIV:
            if (std::abs(std::get<2>(right)) > 1e-5) {
              return std::make_tuple(true, false, std::get<2>(left) / std::get<2>(right));
            } else {
              // Division by zero not allowed.
              return std::make_tuple(false, false, 0);
            }
            break;
          case plansys2_msgs::msg::Node::ARITH_ADD:
            return std::make_tuple(true, false, std::get<2>(left) + std::get<2>(right));
            break;
          case plansys2_msgs::msg::Node::ARITH_SUB:
            return std::make_tuple(true, false, std::get<2>(left) - std::get<2>(right));
            break;
          default:
            break;
        }

        return std::make_tuple(false, false, 0);
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        if (apply && (phase == ApplyPhase::DELETES || phase == ApplyPhase::ADDS)) {
          return std::make_tuple(true, false, 0);  // Written at the end
        }
        if (tree.nodes[node_id].children.size() < 2) {
          // total-cost modifiers carry only the value: costs are not tracked here
          return std::make_tuple(true, true, 0);
        }

        // An object fluent takes an object: read now, written at the end
        const auto & assigned = tree.nodes[tree.nodes[node_id].children[1]];
        if (tree.nodes[node_id].modifier_type == plansys2_msgs::msg::Node::ASSIGN &&
          is_object_node(assigned))
        {
          const auto object = object_of(
            tree, tree.nodes[node_id].children[1], source, functions, use_state);
          if (!object) {
            return std::make_tuple(false, false, 0);  // An unbound parameter
          }
          if (apply && phase == ApplyPhase::NUMERIC && plan) {
            ObjectUpdate update;
            update.function = tree.nodes[tree.nodes[node_id].children[0]];
            update.before = object_of(
              tree, tree.nodes[node_id].children[0], source, functions, use_state).value_or("");
            update.object = *object;
            plan->objects.push_back(update);
          }
          return std::make_tuple(true, true, 0);
        }

        std::tuple<bool, bool, double> left = evaluate_node(
          tree, source, predicates,
          functions, apply, use_state, tree.nodes[node_id].children[0], false, changes, phase,
            plan);
        std::tuple<bool, bool, double> right = evaluate_node(
          tree, source,
          predicates, functions, apply, use_state, tree.nodes[node_id].children[1],
          false, changes, phase, plan);

        if (!std::get<0>(left) || !std::get<0>(right)) {
          // (total-cost) is tracked only if the problem sets it
          const auto & target = tree.nodes[tree.nodes[node_id].children[0]];
          if (target.node_type == plansys2_msgs::msg::Node::FUNCTION &&
            target.name == "total-cost" && target.parameters.empty() && std::get<0>(right))
          {
            return std::make_tuple(true, true, 0);
          }
          return std::make_tuple(false, false, 0);
        }

        bool success = true;
        double value = 0;

        switch (tree.nodes[node_id].modifier_type) {
          case plansys2_msgs::msg::Node::ASSIGN:
            value = std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::INCREASE:
            value = std::get<2>(left) + std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::DECREASE:
            value = std::get<2>(left) - std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::SCALE_UP:
            value = std::get<2>(left) * std::get<2>(right);
            break;
          case plansys2_msgs::msg::Node::SCALE_DOWN:
            // Division by zero not allowed.
            if (std::abs(std::get<2>(right)) > 1e-5) {
              value = std::get<2>(left) / std::get<2>(right);
            } else {
              success = false;
            }
            break;
          default:
            success = false;
            break;
        }

        if (success && apply && phase != ApplyPhase::ALL) {
          // Computed from the state before the action, written at the end
          if (phase == ApplyPhase::NUMERIC && plan) {
            NumericUpdate update;
            update.function = tree.nodes[tree.nodes[node_id].children[0]];
            update.modifier_type = tree.nodes[node_id].modifier_type;
            update.before = std::get<2>(left);
            update.value = value;
            update.delta = value - std::get<2>(left);
            plan->numeric.push_back(update);
          }
        } else if (success && apply) {
          uint32_t left_id = tree.nodes[node_id].children[0];
          if (use_state) {
            auto it =
              std::find_if(
              functions.begin(), functions.end(),
              std::bind(
                &parser::pddl::checkNodeEquality, std::placeholders::_1,
                tree.nodes[left_id], true));
            if (it != functions.end()) {
              it->value = value;
            } else {
              success = false;
            }
          } else {
            // Straight into a node, as fromStringFunction would build it: through text
            // the value lost precision
            plansys2::Function function;
            function.node_type = plansys2_msgs::msg::Node::FUNCTION;
            function.name = tree.nodes[left_id].name;
            function.parameters = tree.nodes[left_id].parameters;
            function.value = value;
            success = source->update(function);
            if (changes && success && value != std::get<2>(left)) {
              FactChange change;
              change.fact = function;
              change.value_before = std::get<2>(left);
              change.value_after = value;
              change.modifier_type = tree.nodes[node_id].modifier_type;
              changes->push_back(change);
            }
          }
        }

        return std::make_tuple(success, false, value);
      }

    case plansys2_msgs::msg::Node::NUMBER: {
        return std::make_tuple(true, true, tree.nodes[node_id].value);
      }

    case plansys2_msgs::msg::Node::CONSTANT: {
        if (tree.nodes[node_id].name.size() > 0) {
          return std::make_tuple(true, true, 0);
        }
        return std::make_tuple(true, false, 0);
      }

    case plansys2_msgs::msg::Node::PARAMETER: {
        if (tree.nodes[node_id].parameters.size() > 0 &&
          tree.nodes[node_id].parameters[0].name.front() != '?')
        {
          return std::make_tuple(true, true, 0);
        }
        [[fallthrough]];
      }

    case plansys2_msgs::msg::Node::EXISTS:
    case plansys2_msgs::msg::Node::FORALL: {
        const auto & node = tree.nodes[node_id];

        // The objects each variable ranges over: those of its type or of a subtype. In the
        // state, an object's types are those its predicates and functions give it; an
        // object of unknown type is not discarded
        std::map<std::string, std::set<std::string>> object_types;
        if (use_state == false) {
          auto instances = source->instances();
          // Without the objects, neither exists nor forall can be decided
          if (!instances.has_value()) {
            return std::make_tuple(false, false, 0);
          }
          for (const auto & instance : instances.value()) {
            object_types[instance.name].insert(instance.type);
          }
        } else {
          auto collect = [&object_types](const plansys2_msgs::msg::Node & n) {
              for (const auto & p : n.parameters) {
                auto & types = object_types[p.name];
                if (!p.type.empty()) {types.insert(p.type);}
              }
            };
          std::for_each(predicates.begin(), predicates.end(), collect);
          std::for_each(functions.begin(), functions.end(), collect);
        }

        std::vector<std::vector<std::string>> parameters_vector;
        for (const auto & var : node.parameters) {
          std::vector<std::string> candidates;
          for (const auto & [name, types] : object_types) {
            bool fits = var.type.empty() || types.empty();
            for (const auto & type : types) {
              fits = fits || type == var.type ||
                std::find(var.sub_types.begin(), var.sub_types.end(), type) != var.sub_types.end();
            }
            if (fits) {candidates.push_back(name);}
          }
          parameters_vector.push_back(candidates);
        }

        // One working copy of the tree: each combination only writes where its variables are
        const uint32_t body_id = node.children[0];
        plansys2_msgs::msg::Tree work = tree;
        std::vector<std::vector<std::pair<uint32_t, size_t>>> places(node.parameters.size());
        collectPlaces(
          tree, body_id, node.parameters, places,
          std::vector<bool>(node.parameters.size(), false));
        auto evaluate_with = [&](const std::vector<std::string> & values, bool apply_body) {
            for (size_t v = 0; v < values.size(); v++) {
              for (const auto & [n, p] : places[v]) {
                work.nodes[n].parameters[p].name = values[v];
              }
            }
            return evaluate_node(
              work, source, predicates, functions, apply_body, use_state, body_id, negate,
              changes, phase, plan);
          };

        // A forall in an effect applies its body for every object
        const bool is_forall = node.node_type == plansys2_msgs::msg::Node::FORALL;
        if (apply && is_forall) {
          bool success = true;
          forEachTuple(
            parameters_vector, [&](const std::vector<std::string> & values) {
              success = std::get<0>(evaluate_with(values, true)) && success;
              return false;
            });
          return std::make_tuple(success, true, 0);
        }

        // The body comes back negated under not, so (not (exists ...)) needs it true for all
        // objects, and (not (forall ...)) for one
        const bool need_all = is_forall != negate;
        bool success = true;
        bool found = false;
        auto visit = [&](const std::vector<std::string> & values) {
            auto result = evaluate_with(values, false);
            success = success && std::get<0>(result);
            found = need_all ? !std::get<1>(result) : std::get<1>(result);
            return found;  // A counterexample, or a witness
          };

        // Looking for a witness of a conjunction among many combinations: they come from the
        // facts of its most selective literal, which the witness has to make true
        size_t combinations = 1;
        for (const auto & c : parameters_vector) {
          combinations = c.empty() ? 0 : std::min<size_t>(combinations * c.size(), 1000000);
        }
        const auto & body = tree.nodes[body_id];
        std::vector<uint32_t> literals;
        if (!need_all && !negate && combinations > 8) {
          if (body.node_type == plansys2_msgs::msg::Node::PREDICATE) {
            literals.push_back(body_id);
          } else if (body.node_type == plansys2_msgs::msg::Node::AND) {
            for (auto child : body.children) {
              if (tree.nodes[child].node_type == plansys2_msgs::msg::Node::PREDICATE) {
                literals.push_back(child);
              }
            }
          }
        }
        std::optional<std::vector<plansys2_msgs::msg::Node>> facts;
        if (!literals.empty()) {
          if (use_state) {
            facts = std::vector<plansys2_msgs::msg::Node>(predicates.begin(), predicates.end());
          } else {
            facts = source->facts();
          }
        }
        if (facts.has_value()) {
          // The literal with fewest facts of its name
          uint32_t best = literals[0];
          size_t best_count = std::numeric_limits<size_t>::max();
          for (auto literal : literals) {
            const auto & l = tree.nodes[literal];
            size_t count = std::count_if(
              facts->begin(), facts->end(), [&](const plansys2_msgs::msg::Node & f) {
                return f.name == l.name && f.parameters.size() == l.parameters.size();
              });
            if (count < best_count) {
              best = literal;
              best_count = count;
            }
          }
          const auto & literal = tree.nodes[best];
          for (const auto & fact : facts.value()) {
            if (fact.name != literal.name || fact.parameters.size() != literal.parameters.size()) {
              continue;
            }
            // The variables the fact binds; the others keep all their candidates
            std::vector<std::vector<std::string>> bound = parameters_vector;
            std::vector<bool> fixed(node.parameters.size(), false);
            bool fits = true;
            for (size_t j = 0; j < literal.parameters.size() && fits; j++) {
              const auto & arg = literal.parameters[j].name;
              const auto & value = fact.parameters[j].name;
              size_t v = 0;
              while (v < node.parameters.size() && node.parameters[v].name != arg) {v++;}
              if (v == node.parameters.size()) {
                fits = arg == value;  // An object: it must be the same
              } else if (fixed[v]) {
                fits = bound[v][0] == value;
              } else {
                fits = std::find(bound[v].begin(), bound[v].end(), value) != bound[v].end();
                bound[v] = {value};
                fixed[v] = true;
              }
            }
            if (fits && forEachTuple(bound, visit)) {
              return std::make_tuple(success, true, 0);
            }
          }
          return std::make_tuple(success, false, 0);
        }

        if (forEachTuple(parameters_vector, visit)) {
          return std::make_tuple(success, !need_all, 0);
        }
        // No object left: everything holds for none, and none exists
        return std::make_tuple(success, need_all, 0);
      }

    // PDDL3: soft, it only counts for the plan; executing, it holds
    case plansys2_msgs::msg::Node::PREFERENCE:
      return std::make_tuple(true, true, 0);

    case plansys2_msgs::msg::Node::IMPLY: {
        // (imply a b) is (or (not a) b); under not, (and a (not b))
        std::tuple<bool, bool, double> left = evaluate_node(
          tree, source, predicates, functions, false, use_state,
          tree.nodes[node_id].children[0], !negate, changes, phase, plan);
        std::tuple<bool, bool, double> right = evaluate_node(
          tree, source, predicates, functions, false, use_state,
          tree.nodes[node_id].children[1], negate, changes, phase, plan);
        const bool truth_value = negate ?
          (std::get<1>(left) && std::get<1>(right)) :
          (std::get<1>(left) || std::get<1>(right));
        return std::make_tuple(std::get<0>(left) && std::get<0>(right), truth_value, 0);
      }

    default:
      std::cerr << "evaluate: Error parsing expresion [" <<
        parser::pddl::toString(tree, node_id) << "]" << std::endl;
  }

  return std::make_tuple(false, false, 0);
}

// Trees may come from outside (e.g. services): walk them only if they are sound
bool sound_tree(const plansys2_msgs::msg::Tree & tree, uint32_t node_id)
{
  std::string error;
  if (node_id >= tree.nodes.size() || !parser::pddl::validateTree(tree, error)) {
    std::cerr << "evaluate: malformed tree: " <<
      (error.empty() ? "node id out of range" : error) << std::endl;
    return false;
  }
  return true;
}

// Applies the effects at once: see ApplyPhase
std::tuple<bool, bool, double> apply_node(
  const plansys2_msgs::msg::Tree & tree,
  FactSource * source,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool use_state,
  uint32_t node_id,
  std::vector<FactChange> * changes,
  bool negate = false)
{
  ApplyPlan plan;
  auto computed = evaluate_node(
    tree, source, predicates, functions, true, use_state, node_id, negate, changes,
    ApplyPhase::NUMERIC, &plan);
  if (!std::get<0>(computed)) {
    return std::make_tuple(false, false, 0);
  }
  auto deleted = evaluate_node(
    tree, source, predicates, functions, true, use_state, node_id, negate, changes,
    ApplyPhase::DELETES, &plan);
  auto added = evaluate_node(
    tree, source, predicates, functions, true, use_state, node_id, negate, changes,
    ApplyPhase::ADDS, &plan);
  bool success = std::get<0>(deleted) && std::get<0>(added);

  // Each function once: its assign or scale (the last one), plus what increases and decreases add
  const auto & numeric = plan.numeric;
  std::vector<bool> written(numeric.size(), false);
  for (size_t i = 0; i < numeric.size(); i++) {
    if (written[i]) {continue;}
    double result = numeric[i].before;
    double added_value = 0.0;
    uint8_t modifier = plansys2_msgs::msg::Node::INCREASE;
    for (size_t j = i; j < numeric.size(); j++) {
      if (written[j] ||
        !parser::pddl::checkNodeEquality(numeric[j].function, numeric[i].function, true))
      {
        continue;
      }
      written[j] = true;
      const auto type = numeric[j].modifier_type;
      if (type == plansys2_msgs::msg::Node::INCREASE ||
        type == plansys2_msgs::msg::Node::DECREASE)
      {
        added_value += numeric[j].delta;
      } else {
        result = numeric[j].value;
        modifier = type;
      }
    }
    const double value = result + added_value;

    if (use_state) {
      auto it = std::find_if(
        functions.begin(), functions.end(),
        std::bind(
          &parser::pddl::checkNodeEquality, std::placeholders::_1, numeric[i].function, true));
      if (it != functions.end()) {
        it->value = value;
      } else {
        success = false;
      }
      continue;
    }

    // Straight into a node, as fromStringFunction would build it: through text
    // the value lost precision
    plansys2::Function function;
    function.node_type = plansys2_msgs::msg::Node::FUNCTION;
    function.name = numeric[i].function.name;
    function.parameters = numeric[i].function.parameters;
    function.value = value;
    const bool updated = source->update(function);
    success = success && updated;
    if (changes && updated && value != numeric[i].before) {
      FactChange change;
      change.fact = function;
      change.value_before = numeric[i].before;
      change.value_after = value;
      change.modifier_type = modifier;
      changes->push_back(change);
    }
  }
  // Object fluents, each with its object; one it did not have yet is added
  for (const auto & update : plan.objects) {
    plansys2::Function function;
    function.node_type = plansys2_msgs::msg::Node::FUNCTION;
    function.name = update.function.name;
    function.parameters = update.function.parameters;
    function.object = update.object;
    if (use_state) {
      auto it = std::find_if(
        functions.begin(), functions.end(),
        std::bind(&parser::pddl::checkNodeEquality, std::placeholders::_1, function, true));
      if (it != functions.end()) {
        it->object = update.object;
      } else {
        functions.push_back(function);
      }
      continue;
    }
    const bool updated = source->update(function) || source->add_function(function);
    success = success && updated;
    if (changes && updated && update.before != update.object) {
      FactChange change;
      change.fact = function;
      change.object_before = update.before;
      change.object_after = update.object;
      change.modifier_type = plansys2_msgs::msg::Node::ASSIGN;
      changes->push_back(change);
    }
  }

  // What an application always gave: the truth of what it applied, and for a modifier the
  // value it computed
  return std::make_tuple(success, std::get<1>(added), std::get<2>(computed));
}

}  // namespace

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  bool use_state,
  uint32_t node_id,
  bool negate)
{
  if (tree.nodes.empty()) {  // No expression
    return std::make_tuple(true, true, 0);
  }

  if (!sound_tree(tree, node_id)) {
    return std::make_tuple(false, false, 0);
  }

  ClientSource source(problem_client);
  if (apply) {
    return apply_node(
      tree, problem_client ? &source : nullptr, predicates, functions, use_state, node_id,
      nullptr, negate);
  }
  return evaluate_node(
    tree, problem_client ? &source : nullptr, predicates, functions, apply, use_state, node_id,
    negate, nullptr);
}

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply,
  uint32_t node_id)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  return evaluate(tree, problem_client, predicates, functions, apply, false, node_id);
}

std::tuple<bool, bool, double> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply,
  uint32_t node_id)
{
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client;
  return evaluate(tree, problem_client, predicates, functions, apply, true, node_id);
}

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, problem_client, false, node_id);

  return std::get<1>(ret);
}

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, predicates, functions, false, node_id);

  return std::get<1>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, problem_client, true, node_id);

  return std::get<0>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<FactChange> & changes,
  uint32_t node_id)
{
  if (tree.nodes.empty()) {
    return true;
  }
  if (!sound_tree(tree, node_id)) {
    return false;
  }

  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  ClientSource source(problem_client);
  auto ret = apply_node(tree, &source, predicates, functions, false, node_id, &changes);
  return std::get<0>(ret);
}

bool check_in_expert(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  if (tree.nodes.empty()) {
    return true;
  }
  return problem_client->isGoalSatisfied(tree);
}

bool check(
  const plansys2_msgs::msg::Tree & tree,
  plansys2::ProblemExpert & expert,
  uint32_t node_id)
{
  if (tree.nodes.empty()) {
    return true;
  }
  if (!sound_tree(tree, node_id)) {
    return false;
  }

  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  ExpertSource source(expert);
  auto ret = evaluate_node(
    tree, &source, predicates, functions, false, false, node_id, false, nullptr);
  return std::get<1>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  plansys2::ProblemExpert & expert,
  uint32_t node_id)
{
  if (tree.nodes.empty()) {
    return true;
  }
  if (!sound_tree(tree, node_id)) {
    return false;
  }
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  ExpertSource source(expert);
  return std::get<0>(apply_node(tree, &source, predicates, functions, false, node_id, nullptr));
}

bool undo(
  const std::vector<FactChange> & changes,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::vector<std::string> * kept)
{
  bool success = true;
  for (auto it = changes.rbegin(); it != changes.rend(); ++it) {
    const auto & change = *it;
    if (change.fact.node_type == plansys2_msgs::msg::Node::PREDICATE) {
      auto exists = problem_client->tryExistPredicate(change.fact);
      if (!exists.has_value()) {
        success = false;
      } else if (exists.value() != change.existed_before) {
        // Otherwise it already is as before, whoever put it back
        success = (change.existed_before ? problem_client->addPredicate(change.fact) :
          problem_client->removePredicate(change.fact)) && success;
      }
      continue;
    }

    std::string expression = "(" + change.fact.name;
    for (const auto & param : change.fact.parameters) {
      expression += " " + param.name;
    }
    expression += ")";

    auto current = problem_client->getFunction(expression);
    if (!current.has_value()) {
      success = false;
      continue;
    }

    // An object fluent: back to its object, if nobody changed it since
    if (!change.object_after.empty()) {
      if (current->object != change.object_after) {
        if (kept) {
          kept->push_back(expression);
        }
        continue;
      }
      plansys2::Function function = change.fact;
      if (change.object_before.empty()) {
        success = problem_client->removeFunction(function) && success;
      } else {
        function.object = change.object_before;
        success = problem_client->updateFunction(function) && success;
      }
      continue;
    }

    plansys2::Function function = change.fact;
    const double tolerance = 1e-9 * std::max(1.0, std::abs(change.value_after));
    const bool additive = change.modifier_type == plansys2_msgs::msg::Node::INCREASE ||
      change.modifier_type == plansys2_msgs::msg::Node::DECREASE;
    if (std::abs(current->value - change.value_after) <= tolerance) {
      function.value = change.value_before;
    } else if (additive) {
      // Changed since by someone else: only this change is taken back
      function.value = current->value - (change.value_after - change.value_before);
    } else {
      // An assignment or a scale overwritten since: the new value stays
      if (kept) {
        kept->push_back(expression);
      }
      continue;
    }
    success = problem_client->updateFunction(function) && success;
  }
  return success;
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id)
{
  std::tuple<bool, bool, double> ret = evaluate(tree, predicates, functions, true, node_id);

  return std::get<0>(ret);
}

std::pair<std::string, int> parse_action(const std::string & input)
{
  std::string action = parser::pddl::getReducedString(input);
  int time = -1;

  size_t delim = action.find(":");
  if (delim != std::string::npos) {
    time = std::stoi(action.substr(delim + 1, action.length() - delim - 1));
    action.erase(action.begin() + delim, action.end());
  }

  action.erase(0, 1);  // remove initial (
  action.pop_back();  // remove last )

  return std::make_pair(action, time);
}

std::string get_action_expression(const std::string & input)
{
  auto action = parse_action(input);
  return action.first;
}

int get_action_time(const std::string & input)
{
  auto action = parse_action(input);
  return action.second;
}

std::string get_action_name(const std::string & input)
{
  auto expr = get_action_expression(input);
  size_t delim = expr.find(" ");
  return expr.substr(0, delim);
}

std::vector<std::string> get_action_params(const std::string & input)
{
  // The tokens after the name: none for (move)
  std::istringstream tokens(get_action_expression(input));
  std::string token;
  tokens >> token;  // The name
  std::vector<std::string> ret;
  while (tokens >> token) {
    ret.push_back(token);
  }
  return ret;
}

plansys2_msgs::msg::Tree replace_children_param(
  const plansys2_msgs::msg::Tree & tree,
  const uint32_t & node_id,
  const std::map<std::string, std::string> & replace)
{
  plansys2_msgs::msg::Tree new_tree = tree;
  if (tree.nodes[node_id].children.size() > 0) {
    for (auto & child_id : tree.nodes[node_id].children) {
      new_tree = replace_children_param(new_tree, child_id, replace);
    }
  }

  for (size_t i = 0; i < tree.nodes[node_id].parameters.size(); i++) {
    if (replace.find(tree.nodes[node_id].parameters[i].name) != replace.end()) {
      new_tree.nodes[node_id].parameters[i].name = replace.at(
        tree.nodes[node_id].parameters[i].name);
    }
  }
  return new_tree;
}

void cart_product(
  std::vector<std::vector<std::string>> & rvvi,  // final result
  std::vector<std::string> & rvi,  // current result
  std::vector<std::vector<std::string>>::const_iterator me,  // current input
  std::vector<std::vector<std::string>>::const_iterator end)  // final input
{
  if (me == end) {
    rvvi.push_back(rvi);
    return;
  }

  const std::vector<std::string> & mevi = *me;
  for (std::vector<std::string>::const_iterator it = mevi.begin(); it != mevi.end(); it++) {
    rvi.push_back(*it);
    cart_product(rvvi, rvi, me + 1, end);
    rvi.pop_back();
  }
}

std::vector<DerivedRule> make_derived_rules(
  const std::vector<plansys2::Derived> & derived,
  const std::vector<plansys2::Instance> & instances,
  const std::function<std::vector<std::string>(const std::string &)> & sub_types)
{
  std::vector<DerivedRule> rules;
  for (const auto & d : derived) {
    DerivedRule rule;
    rule.derived = d;
    for (const auto & param : d.predicate.parameters) {
      // The subtypes the domain gives the parameter, or those asked for
      auto subs = param.sub_types;
      if (subs.empty() && sub_types) {
        subs = sub_types(param.type);
      }
      std::vector<std::string> objects;
      for (const auto & instance : instances) {
        if (instance.type == param.type || param.type.empty() || param.type == "object" ||
          std::find(subs.begin(), subs.end(), instance.type) != subs.end())
        {
          objects.push_back(instance.name);
        }
      }
      rule.objects.push_back(objects);
    }
    rules.push_back(rule);
  }
  return rules;
}

namespace
{
// The predicates a tree reads, each with whether it reads it negated
void read_predicates(
  const plansys2_msgs::msg::Tree & tree, uint32_t id, bool negated,
  std::vector<std::pair<std::string, bool>> & read)
{
  if (id >= tree.nodes.size()) {
    return;
  }
  const auto & node = tree.nodes[id];
  switch (node.node_type) {
    case plansys2_msgs::msg::Node::PREDICATE:
      read.emplace_back(node.name, negated);
      break;
    case plansys2_msgs::msg::Node::NOT:
      for (auto child : node.children) {
        read_predicates(tree, child, !negated, read);
      }
      break;
    case plansys2_msgs::msg::Node::IMPLY:
      // (imply a b) is (or (not a) b)
      if (node.children.size() == 2) {
        read_predicates(tree, node.children[0], !negated, read);
        read_predicates(tree, node.children[1], negated, read);
      }
      break;
    default:
      for (auto child : node.children) {
        read_predicates(tree, child, negated, read);
      }
      break;
  }
}

std::string fact_key(const std::string & name, const std::vector<std::string> & args)
{
  std::string key = name;
  for (const auto & arg : args) {
    key += " " + arg;
  }
  return key;
}

// A literal of a rule body: a predicate, or an equality of two objects; arguments starting with
// '?' are variables
struct Literal
{
  std::string name;  // Empty for an equality
  std::vector<std::string> args;
  bool negated = false;
};

// The body as literals, if it is a conjunction of them (inside exists); false otherwise
bool flatten(
  const plansys2_msgs::msg::Tree & tree, uint32_t id, bool negated, std::vector<Literal> & out)
{
  if (id >= tree.nodes.size()) {
    return false;
  }
  const auto & node = tree.nodes[id];
  auto object = [](const plansys2_msgs::msg::Node & n) -> std::optional<std::string> {
      if (n.node_type == plansys2_msgs::msg::Node::CONSTANT) {
        return n.name;
      }
      if (n.node_type == plansys2_msgs::msg::Node::PARAMETER && !n.parameters.empty()) {
        return n.parameters[0].name;
      }
      return {};
    };
  switch (node.node_type) {
    case plansys2_msgs::msg::Node::AND:
    case plansys2_msgs::msg::Node::EXISTS:
      if (negated) {
        return false;
      }
      for (auto child : node.children) {
        if (!flatten(tree, child, false, out)) {
          return false;
        }
      }
      return true;
    case plansys2_msgs::msg::Node::NOT:
      return !negated && node.children.size() == 1 &&
             flatten(tree, node.children[0], true, out);
    case plansys2_msgs::msg::Node::PREDICATE: {
        Literal literal;
        literal.name = node.name;
        literal.negated = negated;
        for (const auto & param : node.parameters) {
          literal.args.push_back(param.name);
        }
        out.push_back(literal);
        return true;
      }
    case plansys2_msgs::msg::Node::EXPRESSION: {
        if (node.expression_type != plansys2_msgs::msg::Node::COMP_EQ ||
          node.children.size() != 2)
        {
          return false;
        }
        auto first = object(tree.nodes[node.children[0]]);
        auto second = object(tree.nodes[node.children[1]]);
        if (!first || !second) {
          return false;
        }
        Literal literal;
        literal.negated = negated;
        literal.args = {*first, *second};
        out.push_back(literal);
        return true;
      }
    default:
      return false;
  }
}

// The facts so far, by predicate and by key
struct FactIndex
{
  std::map<std::string, std::vector<std::vector<std::string>>> by_name;
  std::set<std::string> keys;

  bool add(const std::string & name, const std::vector<std::string> & args)
  {
    if (!keys.insert(fact_key(name, args)).second) {
      return false;
    }
    by_name[name].push_back(args);
    return true;
  }
};

// What a rule of literals gives: its positive literals joined over the facts, then the negated
// ones and the equalities checked
void join(
  const DerivedRule & rule, const std::vector<Literal> & literals, const FactIndex & facts,
  std::vector<std::vector<std::string>> & heads)
{
  std::vector<const Literal *> positive, others;
  for (const auto & literal : literals) {
    (!literal.name.empty() && !literal.negated ? positive : others).push_back(&literal);
  }
  const size_t arity = rule.derived.predicate.parameters.size();
  std::vector<std::set<std::string>> allowed(arity);
  for (size_t i = 0; i < arity; i++) {
    allowed[i] = std::set<std::string>(rule.objects[i].begin(), rule.objects[i].end());
  }

  std::map<std::string, std::string> binding;
  auto value = [&binding](const std::string & arg) -> std::optional<std::string> {
      if (arg.empty() || arg.front() != '?') {
        return arg;
      }
      auto it = binding.find(arg);
      if (it == binding.end()) {
        return {};
      }
      return it->second;
    };

  // The head, once every variable it has is bound: the ones only in negations range over
  // their objects
  std::function<void(size_t)> finish = [&](size_t i) {
      if (i == arity) {
        for (const auto * literal : others) {
          std::vector<std::string> args;
          for (const auto & arg : literal->args) {
            args.push_back(*value(arg));
          }
          const bool holds = literal->name.empty() ? args[0] == args[1] :
            facts.keys.count(fact_key(literal->name, args)) > 0;
          if (holds == literal->negated) {
            return;
          }
        }
        std::vector<std::string> head;
        for (size_t k = 0; k < arity; k++) {
          head.push_back(binding.at("?" + std::to_string(k)));
        }
        heads.push_back(head);
        return;
      }
      const auto variable = "?" + std::to_string(i);
      auto bound = binding.find(variable);
      if (bound != binding.end()) {
        if (allowed[i].count(bound->second)) {
          finish(i + 1);
        }
        return;
      }
      for (const auto & object : rule.objects[i]) {
        binding[variable] = object;
        finish(i + 1);
      }
      binding.erase(variable);
    };

  std::function<void(size_t)> match = [&](size_t k) {
      if (k == positive.size()) {
        finish(0);
        return;
      }
      const auto & literal = *positive[k];
      auto candidates = facts.by_name.find(literal.name);
      if (candidates == facts.by_name.end()) {
        return;
      }
      for (const auto & args : candidates->second) {
        if (args.size() != literal.args.size()) {
          continue;
        }
        std::vector<std::string> bound_here;
        bool fits = true;
        for (size_t a = 0; a < args.size() && fits; a++) {
          const auto & arg = literal.args[a];
          auto current = value(arg);
          if (current) {
            fits = *current == args[a];
          } else {
            binding[arg] = args[a];
            bound_here.push_back(arg);
          }
        }
        if (fits) {
          match(k + 1);
        }
        for (const auto & arg : bound_here) {
          binding.erase(arg);
        }
      }
    };
  match(0);
}

// Whether a rule of literals can be joined: every variable not of the head in a positive literal
bool joinable(const DerivedRule & rule, const std::vector<Literal> & literals)
{
  std::set<std::string> in_positive;
  for (const auto & literal : literals) {
    if (!literal.name.empty() && !literal.negated) {
      in_positive.insert(literal.args.begin(), literal.args.end());
    }
  }
  const size_t arity = rule.derived.predicate.parameters.size();
  for (const auto & literal : literals) {
    for (const auto & arg : literal.args) {
      if (arg.empty() || arg.front() != '?' || in_positive.count(arg)) {
        continue;
      }
      bool head = false;
      for (size_t i = 0; i < arity && !head; i++) {
        head = arg == "?" + std::to_string(i);
      }
      if (!head) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

std::vector<plansys2::Predicate> derive(
  const std::vector<DerivedRule> & rules,
  const std::vector<plansys2::Predicate> & predicates,
  const std::vector<plansys2::Function> & functions)
{
  if (rules.empty()) {
    return predicates;
  }
  std::set<std::string> names;
  for (const auto & rule : rules) {
    names.insert(rule.derived.predicate.name);
  }

  // Strata: a derived predicate after those it reads, and strictly after those it reads
  // negated, which must be complete before (not q) means anything
  std::map<std::string, size_t> stratum;
  std::vector<std::vector<std::pair<std::string, bool>>> reads(rules.size());
  for (size_t r = 0; r < rules.size(); r++) {
    if (!rules[r].derived.preconditions.nodes.empty()) {
      read_predicates(rules[r].derived.preconditions, 0, false, reads[r]);
    }
  }
  bool stratified = true;
  for (bool changed = true; changed && stratified; ) {
    changed = false;
    for (size_t r = 0; r < rules.size(); r++) {
      const auto & head = rules[r].derived.predicate.name;
      for (const auto & [name, negated] : reads[r]) {
        if (!names.count(name)) {
          continue;
        }
        const size_t needed = stratum[name] + (negated ? 1 : 0);
        if (stratum[head] < needed) {
          stratum[head] = needed;
          changed = true;
          // Deeper than there are predicates: a cycle through a negation
          stratified = stratified && needed <= names.size();
        }
      }
    }
  }
  if (!stratified) {
    std::cerr << "derive: derived predicates negate each other in a cycle: "
      "computed together" << std::endl;
    stratum.clear();
  }
  std::map<size_t, std::vector<const DerivedRule *>> by_stratum;
  for (const auto & rule : rules) {
    by_stratum[stratum[rule.derived.predicate.name]].push_back(&rule);
  }

  // The facts, without what was derived before: it is computed again
  std::vector<plansys2::Predicate> state;
  FactIndex index;
  for (const auto & predicate : predicates) {
    if (!names.count(predicate.name)) {
      state.push_back(predicate);
      std::vector<std::string> args;
      for (const auto & param : predicate.parameters) {
        args.push_back(param.name);
      }
      index.add(predicate.name, args);
    }
  }
  std::vector<plansys2::Function> values = functions;

  auto add = [&](const DerivedRule & rule, const std::vector<std::string> & args) {
      if (!index.add(rule.derived.predicate.name, args)) {
        return false;
      }
      plansys2::Predicate inferred;
      inferred.node_type = plansys2_msgs::msg::Node::PREDICATE;
      inferred.name = rule.derived.predicate.name;
      for (size_t i = 0; i < args.size(); i++) {
        plansys2_msgs::msg::Param param;
        param.name = args[i];
        param.type = rule.derived.predicate.parameters[i].type;
        inferred.parameters.push_back(param);
      }
      state.push_back(inferred);
      return true;
    };

  // Rules of literals are joined over the facts; the rest, tried with every object
  std::vector<std::optional<std::vector<Literal>>> bodies(rules.size());
  for (size_t r = 0; r < rules.size(); r++) {
    std::vector<Literal> literals;
    if (!rules[r].derived.preconditions.nodes.empty() &&
      flatten(rules[r].derived.preconditions, 0, false, literals) &&
      joinable(rules[r], literals))
    {
      bodies[r] = literals;
    }
  }

  // Each stratum, round after round, until nothing new
  for (const auto & [level, level_rules] : by_stratum) {
    (void)level;
    for (bool changed = true; changed; ) {
      changed = false;
      for (const auto * rule : level_rules) {
        if (rule->derived.preconditions.nodes.empty()) {
          continue;
        }
        const auto & body = bodies[rule - rules.data()];
        if (body) {
          std::vector<std::vector<std::string>> heads;
          join(*rule, *body, index, heads);
          for (const auto & head : heads) {
            changed = add(*rule, head) || changed;
          }
          continue;
        }
        std::vector<std::vector<std::string>> combinations;
        std::vector<std::string> combination;
        cart_product(combinations, combination, rule->objects.begin(), rule->objects.end());
        for (const auto & values_of : combinations) {
          if (index.keys.count(fact_key(rule->derived.predicate.name, values_of))) {
            continue;
          }
          std::map<std::string, std::string> replace;
          for (size_t i = 0; i < values_of.size(); i++) {
            replace["?" + std::to_string(i)] = values_of[i];
          }
          auto tree = replace_children_param(
            rule->derived.preconditions, rule->derived.preconditions.nodes[0].node_id, replace);
          if (check(tree, state, values)) {
            changed = add(*rule, values_of) || changed;
          }
        }
      }
    }
  }
  return state;
}

}  // namespace plansys2
