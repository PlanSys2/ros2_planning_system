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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINREADER_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINREADER_HPP_

#include <string>
#include <vector>

namespace plansys2
{

/**
 * @struct plansys2::Domain
 * @brief Structure representing the main components of a PDDL domain.
 *
 * This struct stores the name, requirements, types, constants, predicates, functions,
 * derived predicates, and actions of a PDDL domain.
 */
struct Domain
{
  std::string name;
  std::string requirements;
  std::string types;
  std::string constants;
  std::string predicates;
  std::string functions;
  std::vector<std::string> derived_predicates;
  std::vector<std::string> actions;
};

/**
 * @class plansys2::DomainReader
 * @brief Utility class for parsing and managing multiple PDDL domains.
 *
 * The DomainReader class provides methods to add and parse PDDL domain definitions
 * and extract their components.
 */
class DomainReader
{
public:
  /**
   * @brief Construct a new DomainReader object.
   */
  DomainReader();

  /**
   * @brief Add a new PDDL domain to the internal list.
   *
   * @param[in] domain The PDDL domain definition as a string.
   */
  void add_domain(const std::string & domain);

  /**
   * @brief Get a joint domain definition combining all added domains.
   *
   * @return std::string The combined domain definition.
   */
  std::string get_joint_domain() const;

  /**
   * @brief Get the list of parsed domains.
   *
   * @return std::vector<Domain> Vector containing all parsed domains.
   */
  std::vector<Domain> get_domains() {return domains_;}

protected:
  /**
   * @brief Find the end position of a block in the domain definition.
   *
   * @param[in] domain The domain definition string.
   * @param[in] init_pos Initial position to start searching.
   * @return int The position of the end of the block.
   */
  int get_end_block(const std::string & domain, std::size_t init_pos);

  /**
   * @brief Extract the name of the domain.
   *
   * @param[in,out] domain The domain definition string.
   * @return std::string The name of the domain.
   */
  std::string get_name(std::string & domain);

  /**
   * @brief Extract the requirements section from the domain.
   *
   * @param[in,out] domain The domain definition string.
   * @return std::string The requirements section.
   */
  std::string get_requirements(std::string & domain);

  /**
   * @brief Extract the types defined in the domain.
   *
   * @param[in] domain The domain definition string.
   * @return std::string The types section.
   */
  std::string get_types(const std::string & domain);

  /**
   * @brief Extract the constants defined in the domain.
   *
   * @param[in] domain The domain definition string.
   * @return std::string The constants section.
   */
  std::string get_constants(const std::string & domain);

  /**
   * @brief Extract the predicates defined in the domain.
   *
   * @param[in] domain The domain definition string.
   * @return std::string The predicates section.
   */
  std::string get_predicates(const std::string & domain);

  /**
   * @brief Extract the functions defined in the domain.
   *
   * @param[in] domain The domain definition string.
   * @return std::string The functions section.
   */
  std::string get_functions(const std::string & domain);

  /**
   * @brief Extract the derived predicates defined in the domain.
   *
   * @param[in] domain The domain definition string.
   * @return std::vector<std::string> Vector containing derived predicates.
   */
  std::vector<std::string> get_derived_predicates(const std::string & domain);

  /**
   * @brief Extract the actions defined in the domain.
   *
   * @param[in] domain The domain definition string.
   * @return std::vector<std::string> Vector containing actions.
   */
  std::vector<std::string> get_actions(const std::string & domain);

private:
  // List of parsed domains
  std::vector<Domain> domains_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINREADER_HPP_
