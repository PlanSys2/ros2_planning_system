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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"

#include "plansys2_core/Types.hpp"

namespace plansys2
{

/**
 * @class plansys2::DomainExpertInterface
 * @brief Interface for both DomainExpert and DomainExpertClient classes.
 *
 * This interface defines the methods for querying and manipulating a PDDL domain,
 * including types, constants, predicates, functions, actions, and durative actions.
 */
class DomainExpertInterface
{
public:
  /**
   * @brief Default constructor.
   */
  DomainExpertInterface() {}

  /**
   * @brief Get the domain name.
   *
   * @return std::string The name of the domain.
   */
  virtual std::string getName() = 0;

  /**
   * @brief Get the types defined in the domain.
   *
   * @return std::vector<std::string> The names of the types defined in the domain.
   */
  virtual std::vector<std::string> getTypes() = 0;

  /**
   * @brief Get the constants defined for a given type.
   *
   * @param[in] type The name of the type.
   * @return std::vector<std::string> List of constant names for the specified type.
   */
  virtual std::vector<std::string> getConstants(const std::string & type) = 0;

  /**
   * @brief Get the predicates defined in the domain.
   *
   * @return std::vector<plansys2::Predicate> Vector containing the predicates.
   */
  virtual std::vector<plansys2::Predicate> getPredicates() = 0;

  /**
   * @brief Get the details of a predicate defined in the domain.
   *
   * @param[in] predicate The name of the predicate.
   * @return std::optional<plansys2::Predicate> Predicate object containing the predicate name
   *         and its parameters (name and type).
   *         If the predicate does not exist, the returned value is empty.
   */
  virtual std::optional<plansys2::Predicate> getPredicate(const std::string & predicate) = 0;

  /**
   * @brief Get the functions defined in the domain.
   *
   * @return std::vector<plansys2::Function> Vector containing the functions.
   */
  virtual std::vector<plansys2::Function> getFunctions() = 0;

  /**
   * @brief Get the details of a function defined in the domain.
   *
   * @param[in] function The name of the function.
   * @return std::optional<plansys2::Function> The function name and its parameters (name and type).
   *         If the function does not exist, the returned value is empty.
   */
  virtual std::optional<plansys2::Function> getFunction(const std::string & function) = 0;

  /**
   * @brief Get the derived predicates defined in the domain.
   *
   * @return std::vector<plansys2::Predicate> Vector containing the derived predicates
   *         defined in the domain.
   */
  virtual std::vector<plansys2_msgs::msg::Derived> getDerivedPredicates() = 0;

  /**
   * @brief Get the details of a derived predicate defined in the domain.
   *
   * @param[in] predicate The name of the derived predicate.
   * @param[in] params Optional parameters for the predicate.
   * @return std::vector<plansys2_msgs::msg::Derived> Vector containing the details
   *         of the derived predicate, including its name, parameters (name and type)
   *         and preconditions.
   *         If the derived predicate does not exist, the returned value is empty.
   */
  virtual std::vector<plansys2_msgs::msg::Derived> getDerivedPredicate(
    const std::string & predicate,
    const std::vector<std::string> & params = {}) = 0;

  /**
   * @brief Get the regular actions defined in the domain.
   *
   * @return std::vector<std::string> Vector containing the names of the actions.
   */
  virtual std::vector<std::string> getActions() = 0;

  /**
   * @brief Get the details of a regular action defined in the domain.
   *
   * @param[in] action The name of the action.
   * @param[in] params Optional parameters for the action.
   * @return plansys2_msgs::msg::Action object containing the action name, parameters,
   *         requirements, and effects.
   *         If the action does not exist, the returned value is nullptr.
   */
  virtual plansys2_msgs::msg::Action::SharedPtr getAction(
    const std::string & action, const std::vector<std::string> & params) = 0;

  /**
   * @brief Get the durative actions defined in the domain.
   *
   * @return std::vector<std::string> Vector containing the names of the durative actions.
   */
  virtual std::vector<std::string> getDurativeActions() = 0;

  /**
   * @brief Get the details of a durative action defined in the domain.
   *
   * @param[in] action The name of the durative action.
   * @param[in] params Optional parameters for the action.
   * @return plansys2_msgs::msg::DurativeAction object containing the action name, parameters,
   *         requirements, and effects.
   *         If the action does not exist, the returned value is nullptr.
   */
  virtual plansys2_msgs::msg::DurativeAction::SharedPtr getDurativeAction(
    const std::string & durative_action, const std::vector<std::string> & params) = 0;

  /**
   * @brief Get the current domain definition as a string,
   *        which can be used to save to a file or initialize another domain.
   *
   * @return std::string The current domain definition in PDDL format.
   */
  virtual std::string getDomain() = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTINTERFACE_HPP_
