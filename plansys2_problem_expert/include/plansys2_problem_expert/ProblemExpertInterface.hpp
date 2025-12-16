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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_

#include <vector>
#include <optional>
#include <string>

#include "plansys2_core/Types.hpp"

namespace plansys2
{

/**
 * @class plansys2::ProblemExpertInterface
 * @brief Interface for managing PDDL problem elements.
 *
 * This interface defines the methods for manipulating a PDDL problem,
 * including instances, predicates, unctions, and goals.
 */
class ProblemExpertInterface
{
public:
  /**
   * @brief Default constructor.
   */
  ProblemExpertInterface() {}

  /**
   * @brief Get all instances in the problem.
   *
   * @return std::vector<plansys2::Instance> Vector containing all instances in the problem.
   */
  virtual std::vector<plansys2::Instance> getInstances() = 0;

  /**
   * @brief Add a new instance to the problem.
   *
   * @param[in] instance The instance to be added.
   * @return true if the instance was successfully added, false otherwise.
   */
  virtual bool addInstance(const plansys2::Instance & instance) = 0;

  /**
   * @brief Remove an instance from the problem.
   *
   * @param[in] instance The instance to be removed.
   * @return true if the instance was successfully removed, false otherwise.
   */
  virtual bool removeInstance(const plansys2::Instance & instance) = 0;

  /**
   * @brief Get a specific instance by name.
   *
   * @param[in] name The name of the instance to retrieve.
   * @return std::optional<plansys2::Instance> The instance if found, empty otherwise.
   */
  virtual std::optional<plansys2::Instance> getInstance(const std::string & name) = 0;

  /**
   * @brief Get all predicates in the problem.
   *
   * @return std::vector<plansys2::Predicate> Vector containing all predicates in the problem.
   */
  virtual std::vector<plansys2::Predicate> getPredicates() = 0;

  /**
   * @brief Add a new predicate to the problem.
   *
   * @param[in] predicate The predicate to be added.
   * @return true if the predicate was successfully added, false otherwise.
   */
  virtual bool addPredicate(const plansys2::Predicate & predicate) = 0;

  /**
   * @brief Remove a predicate from the problem.
   *
   * @param[in] predicate The predicate to be removed.
   * @return true if the predicate was successfully removed, false otherwise.
   */
  virtual bool removePredicate(const plansys2::Predicate & predicate) = 0;

  /**
   * @brief Check if a predicate exists in the problem.
   *
   * @param[in] predicate The predicate to check.
   * @return true if the predicate exists, false otherwise.
   */
  virtual bool existPredicate(const plansys2::Predicate & predicate) = 0;

  /**
   * @brief Get a specific predicate by its expression.
   *
   * @param[in] expr The expression of the predicate to retrieve.
   * @return std::optional<plansys2::Predicate> The predicate if found, empty otherwise.
   */
  virtual std::optional<plansys2::Predicate> getPredicate(const std::string & expr) = 0;

  /**
   * @brief Get all functions in the problem.
   *
   * @return std::vector<plansys2::Function> Vector containing all functions in the problem.
   */
  virtual std::vector<plansys2::Function> getFunctions() = 0;

  /**
   * @brief Add a new function to the problem.
   *
   * @param[in] function The function to be added.
   * @return true if the function was successfully added, false otherwise.
   */
  virtual bool addFunction(const plansys2::Function & function) = 0;

  /**
   * @brief Remove a function from the problem.
   *
   * @param[in] function The function to be removed.
   * @return true if the function was successfully removed, false otherwise.
   */
  virtual bool removeFunction(const plansys2::Function & function) = 0;

  /**
   * @brief Check if a function exists in the problem.
   *
   * @param[in] function The function to check.
   * @return true if the function exists, false otherwise.
   */
  virtual bool existFunction(const plansys2::Function & function) = 0;

  /**
   * @brief Update the value of an existing function.
   *
   * @param[in] function The function with the updated value.
   * @return true if the function was successfully updated, false otherwise.
   */
  virtual bool updateFunction(const plansys2::Function & function) = 0;

  /**
   * @brief Get a specific function by its expression.
   *
   * @param[in] expr The expression of the function to retrieve .
   * @return std::optional<plansys2::Function> The function if found, empty otherwise.
   */
  virtual std::optional<plansys2::Function> getFunction(const std::string & expr) = 0;

  /**
   * @brief Get the current goal of the problem.
   *
   * @return plansys2::Goal The current goal.
   */
  virtual plansys2::Goal getGoal() = 0;

  /**
   * @brief Set a new goal for the problem.
   *
   * @param[in] goal The goal to be set.
   * @return true if the goal was successfully set, false otherwise.
   */
  virtual bool setGoal(const plansys2::Goal & goal) = 0;

  /**
   * @brief Check if a goal is satisfied in the current state.
   *
   * @param[in] goal The goal to check for satisfaction.
   * @return true if the goal is satisfied, false otherwise.
   */
  virtual bool isGoalSatisfied(const plansys2::Goal & goal) = 0;

  /**
   * @brief Clear the current goal from the problem.
   *
   * @return true if the goal was successfully cleared, false otherwise.
   */
  virtual bool clearGoal() = 0;

  /**
   * @brief Clear all knowledge (instances, predicates, functions, and goal) from the problem.
   *
   * @return true if the knowledge was successfully cleared, false otherwise.
   */
  virtual bool clearKnowledge() = 0;

  /**
   * @brief Get the complete PDDL problem as a string.
   *
   * @return std::string The complete PDDL problem string.
   */
  virtual std::string getProblem() = 0;

  /**
   * @brief Add a complete PDDL problem from a string.
   *
   * @param[in] problem_str The PDDL problem string to parse and add.
   * @return true if the problem was successfully added, false otherwise.
   */
  virtual bool addProblem(const std::string & problem_str) = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
