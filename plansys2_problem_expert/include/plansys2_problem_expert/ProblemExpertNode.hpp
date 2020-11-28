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
#include "std_msgs/msg/empty.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/add_problem_assignment.hpp"
#include "plansys2_msgs/srv/add_problem_instance.hpp"
#include "plansys2_msgs/srv/add_problem_predicate.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_problem_predicate_details.hpp"
#include "plansys2_msgs/srv/get_problem_predicates.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_msgs/srv/remove_problem_instance.hpp"
#include "plansys2_msgs/srv/remove_problem_predicate.hpp"
#include "plansys2_msgs/srv/exist_problem_predicate.hpp"

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

  void add_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response);

  void add_problem_assignment_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemAssignment::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemAssignment::Response> response);

  void add_problem_instance_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemInstance::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemInstance::Response> response);

  void add_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemPredicate::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemPredicate::Response> response);

  void get_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response);

  void get_problem_instance_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response);

  void get_problem_instances_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Response> response);

  void get_problem_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Response> response);

  void get_problem_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Response> response);

  void get_problem_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response);

  void remove_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response);

  void remove_problem_instance_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemInstance::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemInstance::Response> response);

  void remove_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemPredicate::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemPredicate::Response> response);

  void exist_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ExistProblemPredicate::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ExistProblemPredicate::Response> response);

private:
  std::shared_ptr<ProblemExpert> problem_expert_;

  rclcpp::Service<plansys2_msgs::srv::AddProblemGoal>::SharedPtr
    add_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::AddProblemAssignment>::SharedPtr
    add_problem_assignment_service_;
  rclcpp::Service<plansys2_msgs::srv::AddProblemInstance>::SharedPtr
    add_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AddProblemPredicate>::SharedPtr
    add_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemGoal>::SharedPtr
    get_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
    get_problem_instance_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
    get_problem_instances_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemPredicateDetails>::SharedPtr
    get_problem_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemPredicates>::SharedPtr
    get_problem_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblem>::SharedPtr
    get_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr
    remove_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::RemoveProblemInstance>::SharedPtr
    remove_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::RemoveProblemPredicate>::SharedPtr
    remove_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistProblemPredicate>::SharedPtr
    exist_problem_predicate_service_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
