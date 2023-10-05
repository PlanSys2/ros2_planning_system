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

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

namespace plansys2
{

POPFPlanSolver::POPFPlanSolver()
{
}

void POPFPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;

  arguments_parameter_name_ = plugin_name + ".arguments";
  lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");

  output_dir_parameter_name_ = plugin_name + ".output_dir";
  lc_node_->declare_parameter<std::string>(
    output_dir_parameter_name_, std::filesystem::temp_directory_path());
}

std::optional<plansys2_msgs::msg::Plan>
POPFPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return {};
  }

  auto output_dir = std::filesystem::path(
    lc_node_->get_parameter(output_dir_parameter_name_).value_to_string()
  );

  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_dir /= p;
      }
    }
    std::filesystem::create_directories(output_dir);
  }
  RCLCPP_INFO(
    lc_node_->get_logger(), "Writing planning results to %s.", output_dir.string().c_str());

  plansys2_msgs::msg::Plan ret;

  const auto domain_file_path = output_dir / std::filesystem::path("domain.pddl");
  std::ofstream domain_out(domain_file_path);
  domain_out << domain;
  domain_out.close();

  const auto problem_file_path = output_dir / std::filesystem::path("problem.pddl");
  std::ofstream problem_out(problem_file_path);
  problem_out << problem;
  problem_out.close();

  const auto plan_file_path = output_dir / std::filesystem::path("plan");
  const auto args = lc_node_->get_parameter(arguments_parameter_name_).value_to_string();
  const int status = system(
    ("ros2 run popf popf " + args + " " +
    domain_file_path.string() + " " + problem_file_path.string() + " > " + plan_file_path.string())
    .c_str());

  if (status == -1) {
    return {};
  }

  std::string line;
  std::ifstream plan_file(plan_file_path);
  bool solution = false;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      } else if (line.front() != ';') {
        plansys2_msgs::msg::PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 1);
        duration.pop_back();

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);

        ret.items.push_back(item);
      }
    }
    plan_file.close();
  }

  if (ret.items.empty()) {
    return {};
  }

  return ret;
}

bool
POPFPlanSolver::isDomainValid(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return false;
  }

  auto output_dir = std::filesystem::path(
    lc_node_->get_parameter(output_dir_parameter_name_).value_to_string()
  );

  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_dir /= p;
      }
    }
    std::filesystem::create_directories(output_dir);
  }
  RCLCPP_INFO(
    lc_node_->get_logger(), "Writing domain validation results to %s.",
    output_dir.string().c_str()
  );

  const auto domain_file_path = output_dir / std::filesystem::path("check_domain.pddl");
  std::ofstream domain_out(domain_file_path);
  domain_out << domain;
  domain_out.close();

  const auto problem_file_path = output_dir / std::filesystem::path("check_problem.pddl");
  std::ofstream problem_out(problem_file_path);
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  const auto plan_file_path = output_dir / std::filesystem::path("check.out");
  const int status = system(
    ("ros2 run popf popf " +
    domain_file_path.string() + " " + problem_file_path.string() + " > " + plan_file_path.string())
    .c_str());

  if (status == -1) {
    return false;
  }

  std::string line;
  std::ifstream plan_file(plan_file_path);
  bool solution = false;

  if (plan_file && plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      }
    }
    plan_file.close();
  }

  return solution;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::POPFPlanSolver, plansys2::PlanSolverBase);
