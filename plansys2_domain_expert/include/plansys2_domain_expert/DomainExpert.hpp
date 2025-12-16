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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/DerivedResolutionGraph.hpp"

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_pddl_parser/Domain.hpp"

#include "plansys2_domain_expert/DomainExpertInterface.hpp"
#include "plansys2_domain_expert/DomainReader.hpp"

namespace plansys2
{

/**
 * @class plansys2::DomainExpert
 * @brief Class responsible for managing the internal structure of a PDDL domain.
 *
 * The DomainExpert class allows loading, extending, and querying PDDL domains.
 * It provides methods to retrieve information about types, constants, predicates,
 * functions, actions, and durative actions, as well as to obtain the full domain
 * or check the existence of a specific domain.
 */
class DomainExpert : public DomainExpertInterface
{
public:
  /**
   * @brief Constructor that initializes the DomainExpert with the content of a PDDL domain.
   *
   * @param[in] domain The content of a PDDL domain.
   */
  explicit DomainExpert(const std::string & domain);

  /**
   * @brief Extends the current DomainExpert with another PDDL domain.
   *
   * @param[in] domain The content of a PDDL domain to extend the current one.
   */
  void extendDomain(const std::string & domain);

  /**
   * @brief Get the name of the domain.
   *
   * @return std::string The name of the domain.
   */
  std::string getName();

  /**
   * @brief Get the types defined in the domain.
   *
   * @return std::vector<std::string> The names of the types defined in the domain.
   */
  std::vector<std::string> getTypes();

  /**
   * @brief Get the constants defined for a given type.
   *
   * @param[in] type The name of the type.
   * @return std::vector<std::string> List of constant names for the specified type.
   */
  std::vector<std::string> getConstants(const std::string & type);

  /**
   * @brief Get the predicates defined in the domain.
   *
   * @return std::vector<plansys2::Predicate> Vector containing the predicates defined in the domain.
   */
  std::vector<plansys2::Predicate> getPredicates();

  /**
   * @brief Get the details of a predicate defined in the domain.
   *
   * @param[in] predicate The name of the predicate.
   * @return std::optional<plansys2::Predicate> Predicate object containing the predicate name
   *         and its parameters (name and type).
   *         If the predicate does not exist, the returned value is empty.
   */
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  /**
   * @brief Get the functions defined in the domain.
   *
   * @return std::vector<plansys2::Function> Vector containing the functions defined in the domain.
   */
  std::vector<plansys2::Function> getFunctions();

  /**
   * @brief Get the details of a function defined in the domain.
   *
   * @param[in] function The name of the function.
   * @return std::optional<plansys2::Function> The function name and its parameters (name and type).
   *         If the function does not exist, the returned value is empty.
   */
  std::optional<plansys2::Function> getFunction(const std::string & function);

  /**
   * @brief Get a derived predicate from the domain given an index and parameters.
   *
   * @param derived_index The index of the derived predicate in the domain.
   * @param params The parameters for the derived predicate (empty by default).
   * @return plansys2_msgs::msg::Derived The derived predicate message.
   *
   */
  plansys2_msgs::msg::Derived getDerivedFromDomain(
    const unsigned int & derived_index, const std::vector<std::string> & params = {});

  /// Get the derived predicates existing in the domain.
  /**
   * @brief Get the derived predicates defined in the domain.
   *
   * @return std::vector<plansys2::Predicate> Vector containing the derived predicates
   *         defined in the domain.
   */
  std::vector<plansys2_msgs::msg::Derived> getDerivedPredicates();

  plansys2::DerivedResolutionGraph getDerivedResolutionGraph();

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
  std::vector<plansys2_msgs::msg::Derived> getDerivedPredicate(
    const std::string & predicate,
    const std::vector<std::string> & params = {});

  /**
   * @brief Get the regular actions defined in the domain.
   *
   * @return std::vector<std::string> Vector containing the names of the actions.
   */
  std::vector<std::string> getActions();

  /**
   * @brief Get the details of a regular action defined in the domain.
   *
   * @param[in] action The name of the action.
   * @param[in] params Optional parameters for the action.
   * @return plansys2_msgs::msg::Action object containing the action name, parameters,
   *         requirements, and effects.
   *         If the action does not exist, the returned value is nullptr.
   */
  plansys2_msgs::msg::Action::SharedPtr getAction(
    const std::string & action,
    const std::vector<std::string> & params = {});

  /**
   * @brief Get the durative actions defined in the domain.
   *
   * @return std::vector<std::string> Vector containing the names of the durative actions.
   */
  std::vector<std::string> getDurativeActions();

  /**
   * @brief Get the details of a durative action defined in the domain.
   *
   * @param[in] action The name of the durative action.
   * @param[in] params Optional parameters for the action.
   * @return plansys2_msgs::msg::DurativeAction object containing the action name, parameters,
   *         requirements, and effects.
   *         If the action does not exist, the returned value is nullptr.
   */
  plansys2_msgs::msg::DurativeAction::SharedPtr getDurativeAction(
    const std::string & action,
    const std::vector<std::string> & params = {});

  /**
   * @brief Get the current domain definition as a string,
   *
   *        which can be used to save to a file or initialize another domain.
   * @return std::string The current domain definition in PDDL format.
   */
  std::string getDomain();

  /**
   * @brief Checks if a domain with the given name exists.
   *
   * @param[in] domain_name The name of the domain.
   * @return true if the domain exists, false otherwise.
   */
  bool existDomain(const std::string & domain_name);

private:
  std::shared_ptr<parser::pddl::Domain> domain_;
  DomainReader domains_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
