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

#include "plansys2_planner/PlannerClient.hpp"

#include <chrono>
#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_core/Utils.hpp"

namespace plansys2
{

namespace
{
constexpr auto kAnswerMargin = std::chrono::seconds(5);
}  // namespace

PlannerClient::PlannerClient(ClientNode::Interfaces node)
: node_(std::make_shared<ClientNode>(node)),
  active_(std::make_shared<ActiveComponent>(node_, "planner"))
{
  get_plan_client_ = std::make_shared<ServiceClient<plansys2_msgs::srv::GetPlan>>(
    node_, "planner/get_plan", active_);
  get_plan_array_client_ = std::make_shared<ServiceClient<plansys2_msgs::srv::GetPlanArray>>(
    node_, "planner/get_plan_array", active_);
  // A parameter of the node that has the client; 0: as long as the planner plans, read from it
  // at each request (#434: it was declared uninitialized)
  auto parameters = node.get_node_parameters_interface();
  if (!parameters->has_parameter("plan_solver_timeout")) {
    parameters->declare_parameter("plan_solver_timeout", rclcpp::ParameterValue(0.0));
  }
  planner_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node.get_node_base_interface(), node.get_node_topics_interface(),
    node.get_node_graph_interface(), node.get_node_services_interface(), "planner",
    plansys2::parameters_qos(), node_->group());
}

std::chrono::nanoseconds
PlannerClient::answer_wait()
{
  // The node gone, no own timeout: as long as the planner planned last time
  double timeout = 0.0;
  if (auto node = node_->lock()) {
    timeout = node->get_node_parameters_interface()->get_parameter(
      "plan_solver_timeout").as_double();
  }
  if (timeout <= 0.0) {
    if (wait_for_server(planner_parameters_, std::chrono::seconds(1))) {
      auto future = planner_parameters_->get_parameters({"plan_solver_timeout"});
      if (node_->wait(future, std::chrono::seconds(1)) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        const auto parameters = future.get();
        if (!parameters.empty() &&
          parameters[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
          parameters[0].as_double() > 0.0)
        {
          planner_timeout_ = parameters[0].as_double();
        }
      }
    }
    timeout = planner_timeout_;
  }
  // The planner answers after its own timeout plus the time to stop its solvers (#434)
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout)) + kAnswerMargin;
}

std::optional<plansys2_msgs::msg::Plan>
PlannerClient::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  (void)node_namespace;
  auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
  request->domain = domain;
  request->problem = problem;

  // Active first: answer_wait() asks the planner its timeout
  if (!active_->wait()) {
    return {};
  }
  // The planner answers within answer_wait(): without an answer by then it was lost, and it is
  // asked once more (planning again costs as much as the first time)
  auto response = get_plan_client_->call(request, answer_wait(), 2);
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return result.plan;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_plan_client_->name() << ": " <<
        result.error_info);
    return {};
  }
}

plansys2_msgs::msg::PlanArray
PlannerClient::getPlanArray(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  (void)node_namespace;
  auto request = std::make_shared<plansys2_msgs::srv::GetPlanArray::Request>();
  request->domain = domain;
  request->problem = problem;

  // Active first: answer_wait() asks the planner its timeout
  if (!active_->wait()) {
    return plansys2_msgs::msg::PlanArray();
  }
  // The planner answers within answer_wait(): without an answer by then it was lost, and it is
  // asked once more (planning again costs as much as the first time)
  auto response = get_plan_array_client_->call(request, answer_wait(), 2);
  if (!response) {
    return plansys2_msgs::msg::PlanArray();
  }

  auto result = *response;

  if (result.success) {
    return result.plan_array;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_plan_array_client_->name() << ": " <<
        result.error_info);
    return plansys2_msgs::msg::PlanArray();
  }
}
}  // namespace plansys2
