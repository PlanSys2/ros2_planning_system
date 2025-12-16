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

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
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
   * @brief Clear all knowledge (instances, predicates, functions, and goal) from the problem.
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

private:
  /**
   * @brief Check if all predicates and functions in a tree are valid.
   *
   * @param[in] tree The tree to check.
   * @param[in] domain_expert The domain expert to use for validation.
   * @param[in] node_id The ID of the current node being checked (default: 0).
   * @return true if all predicates and functions in the tree are valid, false otherwise.
   */
  bool checkPredicateTreeTypes(
    const plansys2_msgs::msg::Tree & tree,
    std::shared_ptr<DomainExpert> & domain_expert_,
    uint8_t node_id = 0);

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

  std::vector<plansys2::Instance> instances_;
  std::vector<plansys2::Predicate> predicates_;
  std::vector<plansys2::Function> functions_;
  plansys2::Goal goal_;

  std::shared_ptr<DomainExpert> domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
