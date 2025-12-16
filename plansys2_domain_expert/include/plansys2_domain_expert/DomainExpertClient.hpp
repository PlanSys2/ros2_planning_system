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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/Types.hpp"
#include "plansys2_core/DerivedResolutionGraph.hpp"
#include "plansys2_domain_expert/DomainExpertInterface.hpp"

#include "std_msgs/msg/string.hpp"

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"

#include "plansys2_msgs/srv/get_domain.hpp"
#include "plansys2_msgs/srv/get_domain_name.hpp"
#include "plansys2_msgs/srv/get_domain_types.hpp"
#include "plansys2_msgs/srv/get_domain_constants.hpp"
#include "plansys2_msgs/srv/get_domain_actions.hpp"
#include "plansys2_msgs/srv/get_domain_action_details.hpp"
#include "plansys2_msgs/srv/get_domain_derived_predicate_details.hpp"
#include "plansys2_msgs/srv/get_domain_durative_action_details.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @class plansys2::DomainExpertClient
 * @brief Client for requesting information or sending changes from/to the DomainExpertNode.
 *
 * Any node can create a DomainExpertClient object to request changes to the DomainExpertNode,
 * or to get information from it. It presents the same interface as DomainExpert and hides
 * the complexity of using ROS2 services.
 */
class DomainExpertClient : public DomainExpertInterface
{
public:
  /**
   * @brief Construct a new DomainExpertClient object.
   */
  DomainExpertClient();

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
   * @return std::vector<plansys2::Predicate> Vector containing the predicates.
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
   * @return std::vector<plansys2::Function> Vector containing the functions.
   */
  std::vector<plansys2::Function> getFunctions();

  /**
   * @brief Get the details of a function defined in the domain.
   *
   * @param[in] function The name of the function.
   * @return std::optional<plansys2::Function> Function object with parameters (name and type).
   *         If the function does not exist, the returned value is empty.
   */
  std::optional<plansys2::Function> getFunction(const std::string & function);

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
   *        which can be used to save to a file or initialize another domain.
   *
   * @return std::string The current domain definition in PDDL format.
   */
  std::string getDomain();

  /**
   * @brief Get the current domain definition as a string,
   *        which can be used to save to a file or initialize another domain.
   *
   * @param[in] use_cache If true, use the cached domain if available.
   * @return std::string The current domain definition in PDDL format.
   */
  std::string getDomain(bool use_cache);

  std::string cached_domain_;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<plansys2_msgs::srv::GetDomain>::SharedPtr get_domain_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainName>::SharedPtr get_name_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainTypes>::SharedPtr get_types_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainConstants>::SharedPtr get_constants_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_functions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>::SharedPtr
    get_derived_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>::SharedPtr
    get_derived_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_actions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_durative_actions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr get_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr get_function_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActionDetails>::SharedPtr get_action_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainDurativeActionDetails>::SharedPtr
    get_durative_action_details_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr domain_sub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_
