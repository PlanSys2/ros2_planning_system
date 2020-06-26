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

#include <boost/optional.hpp>

#include <string>
#include <vector>

#include "plansys2_pddl_parser/Domain.h"

#include "plansys2_domain_expert/DomainExpertInterface.hpp"
#include "plansys2_domain_expert/Types.hpp"

namespace plansys2
{

/// DomainExpert is in charge of managing the internal structure of a domain.
class DomainExpert : public DomainExpertInterface
{
public:
  /// Create a new DomainExpert with the content of a domain.
  /**
   * \param[in] node_name The content of a PDDL domain.
   */
  explicit DomainExpert(const std::string & domain);

  /// Extend the content of aDomainExpert with the content of another domain.
  /**
   * \param[in] node_name The content of a PDDL domain.
   */
  void extendDomain(const std::string & domain);

  /// Get the types existing in the domain.
  /**
   * \return The vector containing the names of the types.
   */
  std::vector<std::string> getTypes();

  /// Get the predicates existing in the domain.
  /**
   * \return The vector containing the name of the predicates.
   */
  std::vector<std::string> getPredicates();

  /// Get the details of a predicate existing in the domain.
  /**
   * \param[in] predicate The name of the predicate.
   * \return A Predicate object containing the predicate name andt its parameters (name and type).
   *    If the predicate does not exist, the value returned has not value.
   */
  boost::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  /// Get the regular actions existing in the domain.
  /**
   * \return The vector containing the names of the actions.
   */
  std::vector<std::string> getActions();

  /// Get the details of an regular action existing in the domain.
  /**
   * \param[in] action The name of the action.
   * \return An Action object containing the action name, parameters, requirements and effects.
   *    If the action does not exist, the value returned has not value.
   */
  boost::optional<plansys2::Action> getAction(const std::string & action);

  /// Get the temporal actions existing in the domain.
  /**
   * \return The vector containing the names of the actions.
   */
  std::vector<std::string> getDurativeActions();

  /// Get the details of an durative action existing in the domain.
  /**
   * \param[in] action The name of the action.
   * \return A Durative Action object containing the action name, parameters, requirements and
   *    effects. If the action does not exist, the value returned has not value.
   */
  boost::optional<plansys2::DurativeAction> getDurativeAction(const std::string & action);

  /// Get the current domain, ready to be saved to file, or to initialize another domain.
  /**
   * \return A string containing the domain.
   */
  std::string getDomain();

private:
  parser::pddl::Domain domain_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERT_HPP_
