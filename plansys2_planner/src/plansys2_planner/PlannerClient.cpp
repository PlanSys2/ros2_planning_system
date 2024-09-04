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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

PlannerClient::PlannerClient()
{
  node_ = rclcpp::Node::make_shared("planner_client");

  get_plan_client_ = node_->create_client<plansys2_msgs::srv::GetPlan>("planner/get_plan");
}

std::optional<plansys2_msgs::msg::Plan>
PlannerClient::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  while (!get_plan_client_->wait_for_service(std::chrono::seconds(30))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_plan_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
  request->domain = domain;
  request->problem = problem;

  auto future_result = get_plan_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(15)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.plan;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_plan_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

}  // namespace plansys2
