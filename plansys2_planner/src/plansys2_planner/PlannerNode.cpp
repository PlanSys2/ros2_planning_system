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

#include <string>
#include <memory>
#include <iostream>
#include <fstream>

#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace plansys2
{

PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("planner", options),
  lp_loader_("plansys2_core", "plansys2::PlanSolverBase"),
  default_ids_{},
  default_types_{},
  solver_timeout_(15s)
{
  declare_parameter("plan_solver_plugins", default_ids_);
  double timeout = solver_timeout_.seconds();
  declare_parameter("plan_solver_timeout", timeout);
}

PlannerNode::~PlannerNode()
{
  // Solvers must go before lp_loader_ unloads the libraries that implement them
  solvers_.clear();
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PlannerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  auto node = shared_from_this();
  // Solvers are owned by this node, so they get a non-owning pointer to it: an
  // owning one would keep the node alive forever (#422)
  auto solver_node = rclcpp_lifecycle::LifecycleNode::SharedPtr(
    rclcpp_lifecycle::LifecycleNode::SharedPtr(), this);
  double timeout;

  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  get_parameter("plan_solver_plugins", solver_ids_);
  get_parameter("plan_solver_timeout", timeout);

  solver_timeout_ = rclcpp::Duration((int32_t)timeout, 0);

  if (!solver_ids_.empty()) {
    if (solver_ids_ == default_ids_) {
      for (size_t i = 0; i < default_ids_.size(); ++i) {
        plansys2::declare_parameter_if_not_declared(
          node, default_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_types_[i]));
      }
    }
    solver_types_.resize(solver_ids_.size());

    for (size_t i = 0; i != solver_types_.size(); i++) {
      try {
        solver_types_[i] = plansys2::get_plugin_type_param(node, solver_ids_[i]);
        plansys2::PlanSolverBase::Ptr solver =
          lp_loader_.createUniqueInstance(solver_types_[i]);

        solver->configure(solver_node, solver_ids_[i]);

        RCLCPP_INFO(
          get_logger(), "Created solver : %s of type %s",
          solver_ids_[i].c_str(), solver_types_[i].c_str());
        solvers_.insert({solver_ids_[i], solver});
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(get_logger(), "Failed to create solver. Exception: %s", ex.what());
        exit(-1);
      }
    }
  } else {
    auto default_solver = std::make_shared<plansys2::POPFPlanSolver>();
    default_solver->configure(solver_node, "POPF");
    solvers_.insert({"POPF", default_solver});
    RCLCPP_INFO(
      get_logger(), "Created default solver : %s of type %s",
      "POPF", "plansys2/POPFPlanSolver");
  }

  RCLCPP_INFO(get_logger(), "[%s] Solver Timeout %g", get_name(), solver_timeout_.seconds());

  get_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
    "planner/get_plan",
    std::bind(
      &PlannerNode::get_plan_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_plan_array_service_ = create_service<plansys2_msgs::srv::GetPlanArray>(
    "planner/get_plan_array",
    std::bind(
      &PlannerNode::get_plan_array_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  validate_domain_service_ = create_service<plansys2_msgs::srv::ValidateDomain>(
    "planner/validate_domain",
    std::bind(
      &PlannerNode::validate_domain_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

plansys2_msgs::msg::PlanArray
PlannerNode::get_plan_array(const std::string & domain, const std::string & problem)
{
  std::map<std::string, std::future<std::optional<plansys2_msgs::msg::Plan>>> futures;
  std::map<std::string, std::optional<plansys2_msgs::msg::Plan>> results;

  for (auto & solver : solvers_) {
    futures[solver.first] = std::async(std::launch::async,
      &plansys2::PlanSolverBase::getPlan, solver.second,
      domain, problem, get_namespace(), solver_timeout_);
  }

  auto start = now();

  size_t pending_result = solvers_.size();
  while (pending_result > 0 && now() - start < solver_timeout_) {
    for (auto & fut : futures) {
      if (results.find(fut.first) == results.end()) {
        if (fut.second.wait_for(1ms) == std::future_status::ready) {
          results[fut.first] = fut.second.get();
          pending_result--;
        }
      }
    }
  }

  for (auto & solver : solvers_) {
    if (results.find(solver.first) == results.end()) {
      solver.second->cancel();
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (auto & fut : futures) {
    if (results.find(fut.first) == results.end()) {
      try {
        fut.second.get();
      } catch (const std::exception & e) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Exception while destroying future for "
            << fut.first << ": " << e.what());
      }
    }
  }

  plansys2_msgs::msg::PlanArray plans;
  for (auto & result : results) {
    if (result.second.has_value()) {
      plans.plan_array.push_back(result.second.value());
    }
  }

  std::sort(plans.plan_array.begin(), plans.plan_array.end(),
    [](const plansys2_msgs::msg::Plan & a, const plansys2_msgs::msg::Plan & b)
    {
      return a.items.size() < b.items.size();
    });

  return plans;
}


void
PlannerNode::get_plan_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response)
{
  (void)request_header;
  auto plans = get_plan_array(request->domain, request->problem);

  if (!plans.plan_array.empty()) {
    response->success = true;
    response->plan = plans.plan_array.front();
  } else {
    response->success = false;
    response->error_info = "Plan not found";
  }
}

void
PlannerNode::get_plan_array_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlanArray::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlanArray::Response> response)
{
  (void)request_header;
  response->plan_array = get_plan_array(request->domain, request->problem);

  if (!response->plan_array.plan_array.empty()) {
    response->success = true;
  } else {
    response->success = false;
    response->error_info = "Plan not found";
  }
}

void
PlannerNode::validate_domain_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Response> response)
{
  (void)request_header;
  response->success = solvers_.begin()->second->isDomainValid(
    request->domain, get_namespace());

  if (!response->success) {
    response->error_info = "Domain is not valid";
  }
}

}  // namespace plansys2
