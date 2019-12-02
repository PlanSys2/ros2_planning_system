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

#include "plansys2_domain_expert/DomainExpertInterface.hpp"

#include "plansys2_msgs/srv/get_domain.hpp"
#include "plansys2_msgs/srv/get_domain_types.hpp"
#include "plansys2_msgs/srv/get_domain_predicates.hpp"
#include "plansys2_msgs/srv/get_domain_actions.hpp"
#include "plansys2_msgs/srv/get_domain_predicate_details.hpp"
#include "plansys2_msgs/srv/get_domain_action_details.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

class DomainExpertClient : public DomainExpertInterface
{
public:
  explicit DomainExpertClient(rclcpp::Node::SharedPtr provided_node);

  std::vector<std::string> getTypes();
  std::vector<std::string> getPredicates();
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);
  std::vector<std::string> getActions();
  std::optional<plansys2::Action> getAction(const std::string & action);
  std::vector<std::string> getDurativeActions();
  std::optional<plansys2::DurativeAction> getDurativeAction(const std::string & action);

  std::string getDomain();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<plansys2_msgs::srv::GetDomain>::SharedPtr get_domain_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainTypes>::SharedPtr get_types_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainPredicates>::SharedPtr get_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_actions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainPredicateDetails>::SharedPtr
    get_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetDomainActionDetails>::SharedPtr get_action_details_client_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTCLIENT_HPP_
