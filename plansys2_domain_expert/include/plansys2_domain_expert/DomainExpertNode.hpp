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

#ifndef PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_
#define PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_

#include <memory>

#include "plansys2_domain_expert/DomainExpert.hpp"

#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/srv/get_domain_types.hpp"
#include "plansys2_msgs/srv/get_domain_actions.hpp"
#include "plansys2_msgs/srv/get_domain_action_details.hpp"
#include "plansys2_msgs/srv/get_domain_predicates.hpp"
#include "plansys2_msgs/srv/get_domain_predicate_details.hpp"
#include "plansys2_msgs/srv/get_domain.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

class DomainExpertNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  DomainExpertNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void get_domain_types_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Response> response);

  void get_domain_actions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response);

  void get_domain_action_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Response> response);

  void get_domain_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainPredicates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainPredicates::Response> response);

  void get_domain_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainPredicateDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomainPredicateDetails::Response> response);

  void get_domain_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetDomain::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetDomain::Response> response);

private:
  std::shared_ptr<DomainExpert> domain_expert_;

  rclcpp::Service<plansys2_msgs::srv::GetDomainTypes>::SharedPtr get_types_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainActions>::SharedPtr get_domain_actions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainActionDetails>::SharedPtr
    get_domain_action_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainPredicates>::SharedPtr
    get_domain_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomainPredicateDetails>::SharedPtr
    get_domain_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetDomain>::SharedPtr get_domain_service_;
};

}  // namespace plansys2

#endif  // PLANSYS2_DOMAIN_EXPERT__DOMAINEXPERTNODE_HPP_
