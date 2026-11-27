// Copyright 2026 Intelligent Robotics Lab
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

#include "plansys2_klimax_plan_solver/klimax_plan_solver.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <regex>
#include <string>
#include <utility>

#include "klimax/klimax.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plansys2
{

namespace
{

// Leave time to hand the plan over before the planner node deadline
constexpr std::chrono::milliseconds kMinMargin{100};

std::string action_text(const std::string & name, const std::vector<std::string> & args)
{
  std::string out = "(" + name;
  for (const auto & arg : args) {
    out += " " + arg;
  }
  return out + ")";
}

void log_diagnostics(const rclcpp::Logger & logger, const std::vector<klimax::Diagnostic> & all)
{
  for (const auto & d : all) {
    if (d.severity == klimax::Severity::Error) {
      RCLCPP_ERROR(logger, "%s", d.rendered.c_str());
    } else if (d.severity == klimax::Severity::Warning) {
      RCLCPP_DEBUG(logger, "%s", d.rendered.c_str());
    }
  }
}

// Klimax's plan as PlanSys2's: the actions in the order of execution, and the decomposition
plansys2_msgs::msg::Plan to_msg(const klimax::SolveResult & result)
{
  plansys2_msgs::msg::Plan msg;
  std::map<std::string, const klimax::PlanStep *> step_of;
  for (const auto & step : result.steps_of_plan) {
    step_of[step.id] = &step;
  }
  std::map<std::string, std::string> ref;  // id in Klimax's plan -> in the message
  for (size_t i = 0; i < result.sequence.size(); ++i) {
    const klimax::PlanStep & step = *step_of.at(result.sequence[i]);
    plansys2_msgs::msg::PlanItem item;
    item.action = action_text(step.task, step.args);
    if (result.makespan) {
      item.time = step.start.value_or(0.0);
      item.duration = step.end.value_or(item.time) - item.time;
    } else {
      // A sequential plan: one action after another, as PDDL planners give them
      item.time = static_cast<double>(i);
      item.duration = 0.0;
    }
    msg.items.push_back(std::move(item));
    ref[step.id] = "item:" + std::to_string(i);
  }
  for (const auto & step : result.steps_of_plan) {
    if (!step.primitive) {
      ref[step.id] = step.id;
    }
  }
  auto ordering = [](const std::vector<std::pair<uint32_t, uint32_t>> & order) {
      std::vector<uint32_t> flat;
      for (const auto & [a, b] : order) {
        flat.push_back(a);
        flat.push_back(b);
      }
      return flat;
    };
  // The initial task network first, then every task
  plansys2_msgs::msg::PlanTask root;
  root.id = "root";
  for (const auto & id : result.roots) {
    root.subtasks.push_back(ref.at(id));
  }
  root.ordering = ordering(result.root_order);
  msg.tasks.push_back(std::move(root));
  for (const auto & step : result.steps_of_plan) {
    if (step.primitive) {
      continue;
    }
    plansys2_msgs::msg::PlanTask task;
    task.id = step.id;
    task.task = action_text(step.task, step.args);
    task.method = step.method;
    for (const auto & id : step.subtasks) {
      task.subtasks.push_back(ref.at(id));
    }
    task.ordering = ordering(step.order);
    task.precondition = step.precondition;
    // In the time of the items: in a sequential plan action i is at i, so after k of them is
    // between k - 1 and k
    task.applied = result.makespan ? step.applied_at.value_or(0.0) :
      static_cast<double>(step.applied_after) - 0.5;
    msg.tasks.push_back(std::move(task));
  }
  msg.has_cost = result.cost.has_value();
  msg.cost = result.cost.value_or(0.0);
  return msg;
}

}  // namespace

void KlimaxPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;
  plugin_name_ = plugin_name;
  auto declare = [this](const std::string & name, const rclcpp::ParameterValue & value) {
      if (!lc_node_->has_parameter(param(name))) {
        lc_node_->declare_parameter(param(name), value);
      }
    };
  declare("timeout", rclcpp::ParameterValue(0.0));
  declare("anytime", rclcpp::ParameterValue(false));
}

std::optional<plansys2_msgs::msg::Plan> KlimaxPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace, const rclcpp::Duration solver_timeout)
{
  (void)node_namespace;
  cancel_requested_ = false;
  const auto logger = lc_node_->get_logger().get_child("klimax");

  using std::chrono::milliseconds;
  const auto node_ms = solver_timeout.to_chrono<milliseconds>();
  auto timeout = node_ms - std::max(kMinMargin, node_ms / 20);
  const double param_timeout = lc_node_->get_parameter(param("timeout")).as_double();
  if (param_timeout > 0.0) {
    timeout = std::min(timeout, milliseconds(static_cast<int64_t>(param_timeout * 1000.0)));
  }
  timeout = std::max(timeout, milliseconds(1));

  klimax::SolveOptions options;
  options.seconds = static_cast<double>(timeout.count()) / 1000.0;
  options.anytime = lc_node_->get_parameter(param("anytime")).as_bool();
  options.decomposition = true;
  const klimax::SolveResult result = klimax::solve(domain, problem, options);
  log_diagnostics(logger, result.diagnostics);
  if (result.status != klimax::Status::Solved) {
    RCLCPP_INFO(
      logger, "No plan: %s%s%s", std::string(klimax::to_string(result.status)).c_str(),
      result.message.empty() ? "" : ": ", result.message.c_str());
    return std::nullopt;
  }
  if (cancel_requested_) {
    return std::nullopt;
  }
  return to_msg(result);
}

bool KlimaxPlanSolver::isDomainValid(
  const std::string & domain, const std::string & node_namespace)
{
  (void)node_namespace;
  // With a problem of no tasks, whatever the domain is called: only what is wrong in the
  // domain counts
  std::smatch name;
  const std::string head = domain.substr(0, 2000);
  const std::string domain_name =
    std::regex_search(head, name, std::regex(R"(\(\s*domain\s+([^\s()]+))", std::regex::icase)) ?
    name[1].str() : "d";
  const std::string problem =
    "(define (problem p) (:domain " + domain_name + ") (:htn :subtasks ()) (:init))";
  const auto report = klimax::check(domain, problem, "domain.hddl", "problem.hddl");
  bool valid = true;
  for (const auto & d : report.diagnostics) {
    if (d.severity == klimax::Severity::Error && d.file != "problem.hddl") {
      RCLCPP_ERROR(lc_node_->get_logger().get_child("klimax"), "%s", d.rendered.c_str());
      valid = false;
    }
  }
  return valid;
}

std::set<std::string> KlimaxPlanSolver::supported_requirements() const
{
  std::set<std::string> supported = known_pddl_requirements();
  supported.insert(
    {":hierarchy", ":htn", ":method-preconditions", ":htn-method-prec", ":method-constraints",
      ":durative-methods"});
  return supported;
}

}  // namespace plansys2

PLUGINLIB_EXPORT_CLASS(plansys2::KlimaxPlanSolver, plansys2::PlanSolverBase)
