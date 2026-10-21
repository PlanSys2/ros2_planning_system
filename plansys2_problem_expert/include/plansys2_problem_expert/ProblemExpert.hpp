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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_

#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <memory>

#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

namespace plansys2
{

/**
 * @class plansys2::ProblemExpert
 * @brief Class that implements the ProblemExpertInterface to manage a PDDL problem.
 *
 * This class provides the concrete implementation for manipulating PDDL problem
 * elements such as instances, predicates, functions, and goals.
 */
class ProblemExpert : public ProblemExpertInterface
{
public:
  /**
   * @brief Constructor that initializes the ProblemExpert with a DomainExpert.
   *
   * @param[in] domain_expert Shared pointer to a DomainExpert instance that
   *                          provides information about the PDDL domain.
   */
  explicit ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert);

  /**
   * @brief Get all instances in the problem.
   *
   * @return std::vector<plansys2::Instance> Vector containing all instances in the problem.
   */
  std::vector<plansys2::Instance> getInstances();

  /**
   * @brief Add a new instance to the problem.
   *
   * @param[in] instance The instance to be added.
   * @return true if the instance was successfully added, false otherwise.
   */
  bool addInstance(const plansys2::Instance & instance);

  /**
   * @brief Remove an instance from the problem.
   *
   * @param[in] instance The instance to be removed.
   * @return true if the instance was successfully removed, false otherwise.
   */
  bool removeInstance(const plansys2::Instance & instance);

  /**
   * @brief Get a specific instance by name.
   *
   * @param[in] name The name of the instance to retrieve.
   * @return std::optional<plansys2::Instance> The instance if found, empty otherwise.
   */
  std::optional<plansys2::Instance> getInstance(const std::string & name);

  /**
   * @brief Get all predicates in the problem.
   *
   * @return std::vector<plansys2::Predicate> Vector containing all predicates in the problem.
   */
  std::vector<plansys2::Predicate> getPredicates();

  /**
   * @brief Add a new predicate to the problem.
   *
   * @param[in] predicate The predicate to be added.
   * @return true if the predicate was successfully added, false otherwise.
   */
  bool addPredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Remove a predicate from the problem.
   *
   * @param[in] predicate The predicate to be removed.
   * @return true if the predicate was successfully removed, false otherwise.
   */
  bool removePredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Check if a predicate exists in the problem.
   *
   * @param[in] predicate The predicate to check.
   * @return true if the predicate exists, false otherwise.
   */
  bool existPredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Get a specific predicate by its expression.
   *
   * @param[in] expr The expression of the predicate to retrieve.
   * @return std::optional<plansys2::Predicate> The predicate if found, empty otherwise.
   */
  std::optional<plansys2::Predicate> getPredicate(const std::string & expr);

  /**
   * @brief Get all functions in the problem.
   *
   * @return std::vector<plansys2::Function> Vector containing all functions in the problem.
   */
  std::vector<plansys2::Function> getFunctions();

  /**
   * @brief Add a new function to the problem.
   *
   * @param[in] function The function to be added.
   * @return true if the function was successfully added, false otherwise.
   */
  bool addFunction(const plansys2::Function & function);

  /**
   * @brief Remove a function from the problem.
   *
   * @param[in] function The function to be removed.
   * @return true if the function was successfully removed, false otherwise.
   */
  bool removeFunction(const plansys2::Function & function);

  /**
   * @brief Check if a function exists in the problem.
   *
   * @param[in] function The function to check.
   * @return true if the function exists, false otherwise.
   */
  bool existFunction(const plansys2::Function & function);

  /**
   * @brief Update the value of an existing function.
   *
   * @param[in] function The function with the updated value.
   * @return true if the function was successfully updated, false otherwise.
   */
  bool updateFunction(const plansys2::Function & function);

  /**
   * @brief Get a specific function by its expression.
   *
   * @param[in] expr The expression of the function to retrieve .
   * @return std::optional<plansys2::Function> The function if found, empty otherwise.
   */
  std::optional<plansys2::Function> getFunction(const std::string & expr);

  /**
   * @brief Get the current goal of the problem.
   *
   * @return plansys2::Goal The current goal.
   */
  plansys2::Goal getGoal();

  /**
   * @brief Set a new goal for the problem.
   *
   * @param[in] goal The goal to be set.
   * @return true if the goal was successfully set, false otherwise.
   */
  bool setGoal(const plansys2::Goal & goal);

  /**
   * @brief Check if a goal is satisfied in the current state.
   *
   * @param[in] goal The goal to check for satisfaction.
   * @return true if the goal is satisfied, false otherwise.
   */
  bool isGoalSatisfied(const plansys2::Goal & goal);

  /**
   * @brief Clear the current goal from the problem.
   *
   * @return true if the goal was successfully cleared, false otherwise.
   */
  bool clearGoal();

  /**
   * @brief Get the metric of the problem: what the planner optimizes.
   *
   * @return plansys2::Metric The metric, with an empty tree if there is none.
   */
  plansys2::Metric getMetric();

  /**
   * @brief Set the metric of the problem.
   *
   * @param[in] metric The metric: minimize or maximize a numeric expression over the
   *            functions of the domain and (total-time).
   * @return true if the metric was valid and set, false otherwise.
   */
  bool setMetric(const plansys2::Metric & metric);

  /**
   * @brief Remove the metric of the problem.
   *
   * @return true if the metric was cleared.
   */
  bool clearMetric();

  /**
   * @brief PDDL3: the constraints the plan of the problem keeps, (always (safe))...
   *
   * @return The constraints, an empty tree if there are none.
   */
  plansys2_msgs::msg::Tree getConstraints();

  /**
   * @brief Set the constraints of the problem: modal constraints, in and, forall and
   *        preferences, over the predicates and functions of the domain.
   *
   * @return true if they were valid and set.
   */
  bool setConstraints(const plansys2_msgs::msg::Tree & constraints);

  /**
   * @brief Remove the constraints of the problem.
   */
  bool clearConstraints();

  /**
   * @brief Clear all knowledge (instances, predicates, functions, goal, metric) of the problem.
   *
   * @return true if the knowledge was successfully cleared, false otherwise.
   */
  bool clearKnowledge();

  /**
   * @brief Get the complete PDDL problem as a string.
   *
   * @return std::string The complete PDDL problem string.
   */
  std::string getProblem();

  /**
   * @brief Add a complete PDDL problem from a string.
   *
   * @param[in] problem_str The PDDL problem string to parse and add.
   * @return true if the problem was successfully added, false otherwise.
   */
  bool addProblem(const std::string & problem_str);

  /// A fact that changes at a time: a timed initial literal, or fluent if it is a function
  struct TimedFact
  {
    double at = 0.0;  // When, by the clock
    bool negated = false;  // A predicate that stops holding
    plansys2_msgs::msg::Node fact;  // A predicate, or a function with its value
  };

  /**
   * @brief The clock of the timed facts, in seconds; without one, always 0.
   * @param[in] clock Gives the time now.
   */
  void setClock(std::function<double()> clock) {clock_ = std::move(clock);}

  /**
   * @brief Schedule a fact: a predicate that starts or stops holding, or a function value.
   * @param[in] fact The predicate or the function, with its value.
   * @param[in] delay Seconds from now.
   * @param[in] negated For a predicate, whether it stops holding.
   * @return false if the fact is not valid in the domain, or a function is negated.
   */
  bool addTimedFact(const plansys2_msgs::msg::Node & fact, double delay, bool negated = false);

  /**
   * @brief The facts still to come, by time.
   * @return The facts, with the time each one comes by the clock.
   */
  std::vector<TimedFact> getTimedFacts() const;

  /**
   * @brief Apply the facts whose time has come.
   * @return true if any was applied.
   */
  bool applyTimedFacts();

  /**
   * @brief The continuous effects of a running action, integrated while active.
   * @param[in] id Who they are, the action.
   * @param[in] effects (and (increase (f) (* #t rate))...), bound to its objects.
   * @param[in] active false to stop them.
   * @return false if they are not valid in the domain.
   */
  bool setContinuousEffect(
    const std::string & id, const plansys2_msgs::msg::Tree & effects, bool active);

  /// The continuous effects active now, by who they are
  std::map<std::string, plansys2_msgs::msg::Tree> getContinuousEffects() const
  {
    return continuous_effects_;
  }

  /**
   * @brief Functions perception measures: the dynamics never change them.
   * @param[in] names Their names.
   */
  void setMeasuredFunctions(const std::set<std::string> & names) {measured_functions_ = names;}

  /**
   * @brief One step of the dynamics (PDDL+): the continuous effects of the running actions and
   * the processes whose precondition holds act for dt, and then the events whose precondition
   * holds happen.
   * @param[in] dt Seconds since the last step.
   * @return true if anything changed.
   */
  bool stepDynamics(double dt);

  /**
   * @brief What happened by itself since the last call: the events that fired and the timed
   *        facts applied, as text, "event (float_switch cistern)".
   */
  std::vector<std::string> takeHappened();

  /**
   * @brief Check if an instance with the given name exists.
   *
   * @param[in] name The name of the instance to check.
   * @return true if an instance with the given name exists, false otherwise.
   */
  bool existInstance(const std::string & name);

  /**
   * @brief Check if a type is valid according to the domain.
   *
   * @param[in] type The type to check.
   * @return true if the type is valid, false otherwise.
   */
  bool isValidType(const std::string & type);

  /**
   * @brief Check if a predicate is valid according to the domain.
   *
   * @param[in] predicate The predicate to check.
   * @return true if the predicate is valid, false otherwise.
   */
  bool isValidPredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Check if a function is valid according to the domain.
   *
   * @param[in] function The function to check.
   * @return true if the function is valid, false otherwise.
   */
  bool isValidFunction(const plansys2::Function & function);

  /**
   * @brief Check if a goal is valid according to the domain.
   *
   * @param[in] goal The goal to check.
   * @return true if the goal is valid, false otherwise.
   */
  bool isValidGoal(const plansys2::Goal & goal);

  /**
   * @brief Check if a metric is valid: arithmetic over numbers, (total-time) and functions
   *        of the domain whose arguments are instances of the problem.
   *
   * @param[in] metric The metric to check.
   * @return true if the metric is valid, false otherwise.
   */
  bool isValidMetric(const plansys2::Metric & metric);

  /**
   * @brief Check if constraints are valid: modal constraints over valid conditions.
   */
  bool isValidConstraints(const plansys2_msgs::msg::Tree & constraints);

  /**
   * @brief The names of the preferences of the goal and the constraints: what a metric can
   *        read with (is-violated name).
   */
  std::set<std::string> getPreferenceNames();

  /**
   * @brief Replace the domain this problem is checked against, given as PDDL content.
   *        Knowledge (instances, predicates, functions, goal, metric) that is no longer
   *        consistent with the new domain is pruned; everything that remains valid
   *        is left untouched. If the PDDL is invalid, nothing is changed.
   *
   * @param[in] domain The new domain, in PDDL.
   * @return true if the domain was replaced, false if the PDDL was invalid.
   */
  /**
   * @brief The domain the problem is checked against, as text.
   *
   * @return std::string The joint domain.
   */
  std::string getDomain();

  bool updateDomain(const std::string & domain);

private:
  // The derived predicates of the domain over the instances of the problem
  std::vector<DerivedRule> derivedRules();

  /**
   * @brief Check if all predicates and functions in a tree are valid.
   *
   * @param[in] tree The tree to check.
   * @param[in] domain_expert The domain expert to use for validation.
   * @param[in] node_id The ID of the current node being checked (default: 0).
   * @param[in] bound Variables of the exists and forall above the node, by name.
   * @return true if all predicates and functions in the tree are valid, false otherwise.
   */
  bool checkPredicateTreeTypes(
    const plansys2_msgs::msg::Tree & tree,
    std::shared_ptr<DomainExpert> & domain_expert_,
    uint32_t node_id = 0,
    const std::map<std::string, plansys2_msgs::msg::Param> & bound = {});

  /**
   * @brief Check the arguments of a predicate or function against its model in the domain:
   *        instances or constants of the expected type, or variables bound by exists/forall.
   *
   * @param[in] node The predicate or function.
   * @param[in] model The predicate or function as the domain declares it.
   * @param[in] bound Variables of the exists and forall above the node, by name.
   * @return true if every argument fits, false otherwise.
   */
  bool argumentsFit(
    const plansys2_msgs::msg::Node & node, const plansys2_msgs::msg::Node & model,
    const std::map<std::string, plansys2_msgs::msg::Param> & bound = {});

  /**
   * @brief Remove predicates that reference a specific instance.
   *
   * @param[in,out] predicates The predicates to filter.
   * @param[in] instance The instance to check for references.
   */
  void removeInvalidPredicates(
    std::vector<plansys2::Predicate> & predicates,
    const plansys2::Instance & instance);

  /**
   * @brief Remove functions that reference a specific instance.
   *
   * @param[in,out] functions The functions to filter.
   * @param[in] instance The instance to check for references.
   */
  void removeInvalidFunctions(
    std::vector<plansys2::Function> & functions,
    const plansys2::Instance & instance);

  /**
   * @brief Remove goals that reference a specific instance.
   *
   * @param[in] instance The instance to check for references.
   */
  void removeInvalidGoals(const plansys2::Instance & instance);

  /**
   * @brief A copy of a predicate or function whose parameters get the type of their
   *        instance.
   *
   * @param[in] node The predicate or function.
   * @return plansys2_msgs::msg::Node The node with the types of its instances.
   */
  plansys2_msgs::msg::Node withInstanceTypes(const plansys2_msgs::msg::Node & node);

  /**
   * @brief A copy of a goal whose exists and forall variables get the subtypes of their type.
   *
   * @param[in] goal The goal.
   * @return plansys2::Goal The goal with the subtypes from the domain.
   */
  plansys2::Goal withSubTypes(const plansys2::Goal & goal);

  std::vector<plansys2::Instance> instances_;
  std::vector<plansys2::Predicate> predicates_;
  std::vector<plansys2::Function> functions_;
  plansys2::Goal goal_;
  plansys2::Metric metric_;
  plansys2_msgs::msg::Tree constraints_;
  std::vector<TimedFact> timed_facts_;
  std::map<std::string, plansys2_msgs::msg::Tree> continuous_effects_;
  std::set<std::string> measured_functions_;
  // Processes and events over the instances, made again when the instances change
  std::string happenings_instances_;
  std::vector<plansys2_msgs::msg::Action> processes_;
  std::vector<plansys2_msgs::msg::Action> events_;
  std::vector<std::string> happened_;
  void groundHappenings();
  std::function<double()> clock_ {[]() {return 0.0;}};

  std::shared_ptr<DomainExpert> domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
