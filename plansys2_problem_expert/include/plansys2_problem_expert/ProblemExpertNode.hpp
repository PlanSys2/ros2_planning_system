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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_

#include <memory>

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/srv/get_problem_types.hpp"
#include "plansys2_msgs/srv/get_problem_actions.hpp"
#include "plansys2_msgs/srv/get_problem_action_details.hpp"
#include "plansys2_msgs/srv/get_problem_predicates.hpp"
#include "plansys2_msgs/srv/get_problem_predicate_details.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

class ProblemExpertNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ProblemExpertNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void get_problem_types_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemTypes::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemTypes::Response> response);

  void get_problem_actions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemActions::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemActions::Response> response);

  void get_problem_action_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemActionDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemActionDetails::Response> response);

  void get_problem_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Response> response);

  void get_problem_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Response> response);

  void get_problem_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response);

private:
  std::shared_ptr<ProblemExpert> problem_expert_;

  rclcpp::Service<plansys2_msgs::srv::GetProblemTypes>::SharedPtr get_types_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemActions>::SharedPtr get_problem_actions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemActionDetails>::SharedPtr
    get_problem_action_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemPredicates>::SharedPtr
    get_problem_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemPredicateDetails>::SharedPtr
    get_problem_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblem>::SharedPtr get_problem_service_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
