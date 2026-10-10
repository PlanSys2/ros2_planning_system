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

#include <algorithm>
#include <chrono>
#include <future>
#include <map>
#include <set>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>

#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
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

  // Fractions of a second count: 0.5 used to mean no time at all (#434)
  if (timeout <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "plan_solver_timeout must be positive (%g), using 15 seconds", timeout);
    timeout = 15.0;
  }
  solver_timeout_ = rclcpp::Duration::from_seconds(timeout);

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
      } catch (const std::exception & ex) {
        // The transition fails instead of the whole process (#434)
        RCLCPP_ERROR(
          get_logger(), "Failed to create solver %s: %s", solver_ids_[i].c_str(), ex.what());
        solvers_.clear();
        return CallbackReturnT::FAILURE;
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
  std::string error_info;
  return solve(domain, problem, error_info);
}

plansys2_msgs::msg::PlanArray
PlannerNode::solve(
  const std::string & domain, const std::string & problem, std::string & error_info)
{
  std::map<std::string, std::future<std::optional<plansys2_msgs::msg::Plan>>> futures;
  for (auto & solver : solvers_) {
    futures[solver.first] = std::async(std::launch::async,
      &plansys2::PlanSolverBase::getPlan, solver.second,
      domain, problem, get_namespace(), solver_timeout_);
  }

  // Wall time, whatever clock the node uses
  const auto deadline = std::chrono::steady_clock::now() +
    solver_timeout_.to_chrono<std::chrono::nanoseconds>();
  std::set<std::string> timed_out;
  for (auto & [id, future] : futures) {
    if (future.wait_until(deadline) != std::future_status::ready) {
      solvers_.at(id)->cancel();
      timed_out.insert(id);
    }
  }

  plansys2_msgs::msg::PlanArray plans;
  std::vector<std::string> reasons;
  for (auto & [id, future] : futures) {
    // A solver error must not escape the service callback (#434)
    try {
      auto plan = future.get();
      if (plan.has_value()) {
        plans.plan_array.push_back(plan.value());
      } else if (timed_out.count(id)) {
        reasons.push_back(
          id + " timed out after " + std::to_string(solver_timeout_.seconds()) + " s");
      } else {
        reasons.push_back(id + " found no plan");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Solver %s failed: %s", id.c_str(), e.what());
      reasons.push_back(id + " failed: " + e.what());
    }
  }

  std::sort(plans.plan_array.begin(), plans.plan_array.end(),
    [](const plansys2_msgs::msg::Plan & a, const plansys2_msgs::msg::Plan & b)
    {
      return a.items.size() < b.items.size();
    });

  if (plans.plan_array.empty()) {
    // Only now, so the check never costs anything when there is a plan
    error_info = check_pddl(domain, problem);
    if (error_info.empty()) {
      error_info = "Plan not found";
      for (size_t i = 0; i < reasons.size(); i++) {
        error_info += (i == 0 ? ": " : "; ") + reasons[i];
      }
    }
  }

  return plans;
}

std::string
PlannerNode::check_pddl(const std::string & domain, const std::string & problem)
{
  parser::pddl::Domain parsed_domain;
  try {
    parsed_domain.parse(domain);
  } catch (const std::exception & e) {
    return std::string("Invalid PDDL domain: ") + e.what();
  }
  try {
    parser::pddl::Instance parsed_problem(parsed_domain);
    parsed_problem.parse(problem);
  } catch (const std::exception & e) {
    return std::string("Invalid PDDL problem: ") + e.what();
  }
  return "";
}

void
PlannerNode::get_plan_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response)
{
  (void)request_header;
  auto plans = solve(request->domain, request->problem, response->error_info);

  if (!plans.plan_array.empty()) {
    response->success = true;
    response->plan = plans.plan_array.front();
  } else {
    response->success = false;
  }
}

void
PlannerNode::get_plan_array_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlanArray::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlanArray::Response> response)
{
  (void)request_header;
  response->plan_array = solve(request->domain, request->problem, response->error_info);
  response->success = !response->plan_array.plan_array.empty();
}

void
PlannerNode::validate_domain_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Response> response)
{
  (void)request_header;
  try {
    response->success = solvers_.begin()->second->isDomainValid(
      request->domain, get_namespace());
    if (!response->success) {
      response->error_info = "Domain is not valid";
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Domain check failed: %s", e.what());
    response->success = false;
    response->error_info = std::string("Domain check failed: ") + e.what();
  }
}

}  // namespace plansys2
