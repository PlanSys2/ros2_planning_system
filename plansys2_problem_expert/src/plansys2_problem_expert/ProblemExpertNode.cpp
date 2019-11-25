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

#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include <string>
#include <memory>

#include "lifecycle_msgs/msg/state.hpp"

namespace plansys2
{

ProblemExpertNode::ProblemExpertNode()
: rclcpp_lifecycle::LifecycleNode("problem_expert")
{
  declare_parameter("model_file", "");

  get_types_service_ = create_service<plansys2_msgs::srv::GetProblemTypes>(
    "~/get_problem_types", std::bind(&ProblemExpertNode::get_problem_types_service_callback,
    this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));
  get_problem_actions_service_ = create_service<plansys2_msgs::srv::GetProblemActions>(
    "~/get_problem_actions", std::bind(&ProblemExpertNode::get_problem_actions_service_callback,
    this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));
  get_problem_action_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemActionDetails>(
    "~/get_problem_action_details", std::bind(
      &ProblemExpertNode::get_problem_action_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_problem_predicates_service_ = create_service<plansys2_msgs::srv::GetProblemPredicates>(
    "~/get_problem_predicates", std::bind(
      &ProblemExpertNode::get_problem_predicates_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_problem_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemPredicateDetails>(
    "~/get_problem_predicate_details", std::bind(
      &ProblemExpertNode::get_problem_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
  get_problem_service_ = create_service<plansys2_msgs::srv::GetProblem>(
    "~/get_problem", std::bind(&ProblemExpertNode::get_problem_service_callback,
    this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ProblemExpertNode::on_configure(const rclcpp_lifecycle::State & state)
{
  std::string model_file = get_parameter("model_file").get_value<std::string>();
  problem_expert_ = std::make_shared<ProblemExpert>(model_file);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_activate(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_error(const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

void
ProblemExpertNode::get_problem_types_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemTypes::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemTypes::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->types = problem_expert_->getTypes();
  }
}

void
ProblemExpertNode::get_problem_actions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemActions::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemActions::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->actions = problem_expert_->getActions();
  }
}

void
ProblemExpertNode::get_problem_action_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemActionDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemActionDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto params = problem_expert_->getActionParams(request->action);
    if (params.has_value()) {
      response->success = true;
      response->argument_params = params.value();
    } else {
      RCLCPP_WARN(get_logger(), "Requesting a non-existing action [%s]", request->action.c_str());
      response->success = false;
      response->error_info = "Action not found";
    }
  }
}

void
ProblemExpertNode::get_problem_predicates_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->predicates = problem_expert_->getPredicates();
  }
}

void
ProblemExpertNode::get_problem_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto params = problem_expert_->getPredicateParams(request->predicate);
    if (params.has_value()) {
      response->success = true;
      response->argument_params = params.value();
    } else {
      RCLCPP_WARN(get_logger(), "Requesting a non-existing predicate [%s]",
        request->predicate.c_str());
      response->success = false;
      response->error_info = "Predicate not found";
    }
  }
}

void
ProblemExpertNode::get_problem_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;

    std::ostringstream stream;
    stream << problem_expert_->getProblem();
    response->problem = stream.str();
  }
}


}  // namespace plansys2
