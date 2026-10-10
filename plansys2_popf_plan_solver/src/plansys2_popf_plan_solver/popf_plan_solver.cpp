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


#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <string>
#include <system_error>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"
#include "rclcpp/logging.hpp"

namespace plansys2
{

namespace
{

// The domain check uses a trivial problem: popf answers at once unless something is wrong
constexpr auto kDomainCheckTimeout = 15s;

// Writes content to path, reporting whether it all got to disk (#441)
bool write_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream out(path);
  out << content;
  out.close();
  return !out.fail();
}

std::string shell_quoted(const std::string & text)
{
  std::ostringstream os;
  os << std::quoted(text);
  return os.str();
}

}  // namespace

POPFPlanSolver::POPFPlanSolver()
{
}

std::optional<std::filesystem::path>
POPFPlanSolver::create_folders(const std::string & node_namespace)
{
  auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();

  // Allow usage of the HOME directory with the `~` character; HOME is only needed then
  if (!output_dir.empty() && output_dir[0] == '~') {
    const char * home_dir = std::getenv("HOME");
    if (!home_dir) {
      RCLCPP_ERROR(
        lc_node_->get_logger(), "HOME is not set: cannot expand ~ in %s", output_dir.c_str());
      return std::nullopt;
    }
    output_dir.replace(0, 1, home_dir);
  }

  // Create the necessary folders, returning if there is an error.
  auto output_path = std::filesystem::path(output_dir);
  for (auto p : std::filesystem::path(node_namespace) ) {
    if (p != std::filesystem::current_path().root_directory()) {
      output_path /= p;
    }
  }
  try {
    std::filesystem::create_directories(output_path);
  } catch (std::filesystem::filesystem_error & err) {
    RCLCPP_ERROR(lc_node_->get_logger(), "Error writing directories: %s", err.what());
    return std::nullopt;
  }
  return output_path;
}

std::optional<std::filesystem::path>
POPFPlanSolver::create_run_folder(const std::string & node_namespace)
{
  const auto output_dir = create_folders(node_namespace);
  if (!output_dir) {
    return std::nullopt;
  }

  std::string run_template = (output_dir.value() / "popf_XXXXXX").string();
  if (mkdtemp(run_template.data()) == nullptr) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Cannot create a folder in %s: %s",
      output_dir.value().c_str(), std::strerror(errno));
    return std::nullopt;
  }
  return std::filesystem::path(run_template);
}

void
POPFPlanSolver::remove_run_folder(const std::filesystem::path & run_folder)
{
  if (lc_node_->get_parameter(keep_files_parameter_name_).as_bool()) {
    RCLCPP_DEBUG(lc_node_->get_logger(), "popf files kept in %s", run_folder.c_str());
    return;
  }
  std::error_code error;
  std::filesystem::remove_all(run_folder, error);
}

std::string
POPFPlanSolver::popf_command(
  const std::filesystem::path & domain_path, const std::filesystem::path & problem_path)
{
  const auto args = lc_node_->get_parameter(arguments_parameter_name_).value_to_string();
  return shell_quoted(popf_path_) + " " + args + " " + shell_quoted(domain_path.string()) + " " +
         shell_quoted(problem_path.string());
}

void POPFPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;

  arguments_parameter_name_ = plugin_name + ".arguments";
  output_dir_parameter_name_ = plugin_name + ".output_dir";
  keep_files_parameter_name_ = plugin_name + ".keep_files";

  if (!lc_node_->has_parameter(arguments_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");
  }
  if (!lc_node_->has_parameter(output_dir_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      output_dir_parameter_name_, std::filesystem::temp_directory_path());
  }
  if (!lc_node_->has_parameter(keep_files_parameter_name_)) {
    lc_node_->declare_parameter<bool>(keep_files_parameter_name_, false);
  }

  // Run popf directly: `ros2 run` added the CLI start-up to every plan (#441)
  try {
    std::filesystem::path prefix;
    ament_index_cpp::get_package_prefix("popf", prefix);
    popf_path_ = (prefix / "lib" / "popf" / "popf").string();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      lc_node_->get_logger(), "popf package not found (%s), using popf from PATH", e.what());
    popf_path_ = "popf";
  }
}

std::optional<plansys2_msgs::msg::Plan>
POPFPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace,
  const rclcpp::Duration solver_timeout)
{
  const auto run_folder = create_run_folder(node_namespace);
  if (!run_folder) {
    return {};
  }
  RCLCPP_DEBUG(
    lc_node_->get_logger(), "Writing planning results to %s.", run_folder.value().c_str());

  const auto domain_file_path = run_folder.value() / "domain.pddl";
  const auto problem_file_path = run_folder.value() / "problem.pddl";
  const auto plan_file_path = run_folder.value() / "plan";

  std::optional<plansys2_msgs::msg::Plan> plan;
  if (!write_file(domain_file_path, domain) || !write_file(problem_file_path, problem)) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Cannot write the domain and problem to %s",
      run_folder.value().c_str());
  } else {
    const auto command = popf_command(domain_file_path, problem_file_path);
    RCLCPP_DEBUG(
      lc_node_->get_logger(), "[%s-popf] called with timeout %f seconds: %s",
      lc_node_->get_name(), solver_timeout.seconds(), command.c_str());

    if (execute_planner(command, solver_timeout, plan_file_path.string())) {
      plan = parse_plan_result(plan_file_path.string());
    }
  }

  remove_run_folder(run_folder.value());
  return plan;
}

std::optional<plansys2_msgs::msg::Plan>
POPFPlanSolver::parse_plan_result(const std::string & plan_path)
{
  // "<time>: (<action>)  [<duration>]"; anything else after "Solution Found" is ignored (#441)
  static const std::regex item_regex(
    R"(^\s*(\d+(?:\.\d*)?)\s*:\s*(\(.*\))\s*\[\s*(\d+(?:\.\d*)?)\s*\]\s*$)");

  std::string line;
  std::ifstream plan_file(plan_path);
  bool solution = false;

  plansys2_msgs::msg::Plan plan;

  while (plan_file && std::getline(plan_file, line)) {
    if (!solution) {
      solution = line.find("Solution Found") != std::string::npos;
      continue;
    }

    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == ';') {
      continue;
    }

    std::smatch match;
    if (!std::regex_match(line, match, item_regex)) {
      RCLCPP_WARN(
        lc_node_->get_logger(), "Ignoring unexpected line in the popf plan: %s", line.c_str());
      continue;
    }

    plansys2_msgs::msg::PlanItem item;
    item.time = std::stof(match[1].str());
    item.action = match[2].str();
    item.duration = std::stof(match[3].str());
    plan.items.push_back(item);
  }

  if (solution) {
    return plan;
  } else {
    return {};
  }
}

bool
POPFPlanSolver::isDomainValid(
  const std::string & domain,
  const std::string & node_namespace)
{
  const auto run_folder = create_run_folder(node_namespace);
  if (!run_folder) {
    return false;
  }
  RCLCPP_DEBUG(
    lc_node_->get_logger(), "Writing domain validation results to %s.",
    run_folder.value().c_str());

  // popf with a void problem: it only finds a solution if it accepts the domain
  const auto domain_file_path = run_folder.value() / "check_domain.pddl";
  const auto problem_file_path = run_folder.value() / "check_problem.pddl";
  const auto plan_file_path = run_folder.value() / "check.out";

  bool valid = false;
  if (!write_file(domain_file_path, domain) ||
    !write_file(problem_file_path, "(define (problem void) (:domain plansys2))"))
  {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Cannot write the domain to %s", run_folder.value().c_str());
  } else if (execute_planner(  // NOLINT
      // Without the plugin arguments: they are planning options, not part of the check
      shell_quoted(popf_path_) + " " + shell_quoted(domain_file_path.string()) + " " +
      shell_quoted(problem_file_path.string()),
      kDomainCheckTimeout, plan_file_path.string()))
  {
    std::ifstream plan_file(plan_file_path);
    std::string line;
    while (!valid && std::getline(plan_file, line)) {
      valid = line.find("Solution Found") != std::string::npos;
    }
  }

  remove_run_folder(run_folder.value());
  return valid;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::POPFPlanSolver, plansys2::PlanSolverBase);
