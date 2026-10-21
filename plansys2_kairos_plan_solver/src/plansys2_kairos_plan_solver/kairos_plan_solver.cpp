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

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "kairos/kairos.hpp"
#include "plansys2_core/PddlRequirements.hpp"
#include "plansys2_kairos_plan_solver/kairos_plan_solver.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace plansys2
{

namespace
{

// Time kept to return the plan before the planner node gives up on the call
constexpr auto kMinMargin = std::chrono::milliseconds(20);
// How often a running search checks for cancel()
constexpr auto kCancelPoll = std::chrono::milliseconds(2);

std::optional<kairos::LogLevel> log_level(const std::string & name)
{
  if (name == "debug") {
    return kairos::LogLevel::Debug;
  }
  if (name == "info") {
    return kairos::LogLevel::Info;
  }
  if (name == "warn" || name == "warning") {
    return kairos::LogLevel::Warning;
  }
  if (name == "error") {
    return kairos::LogLevel::Error;
  }
  return std::nullopt;
}

std::optional<kairos::Engine> engine(const std::string & name)
{
  if (name == "auto") {
    return kairos::Engine::Auto;
  }
  if (name == "m1") {
    return kairos::Engine::M1;
  }
  if (name == "m2") {
    return kairos::Engine::M2;
  }
  if (name == "m3") {
    return kairos::Engine::M3;
  }
  return std::nullopt;
}

void log_diagnostics(
  const rclcpp::Logger & logger, const std::vector<kairos::Diagnostic> & diagnostics,
  std::string_view source = {})
{
  for (const auto & d : diagnostics) {
    const auto text = kairos::to_string(d, source);
    switch (d.severity) {
      case kairos::Severity::Error:
        RCLCPP_ERROR(logger, "%s", text.c_str());
        break;
      case kairos::Severity::Warning:
        RCLCPP_WARN(logger, "%s", text.c_str());
        break;
      case kairos::Severity::Note:
        RCLCPP_DEBUG(logger, "%s", text.c_str());
        break;
    }
  }
}

// Writes content to path, reporting whether it all got to disk
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

// Seconds with a point whatever the locale, as the kairos executable reads them
std::string seconds(double value)
{
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os << value;
  return os.str();
}

plansys2_msgs::msg::Plan to_msg(const kairos::Plan & plan)
{
  plansys2_msgs::msg::Plan msg;
  msg.items.reserve(plan.steps.size());
  for (const auto & step : plan.steps) {
    plansys2_msgs::msg::PlanItem item;
    item.time = step.time;
    item.action = step.action();
    item.duration = step.duration;
    msg.items.push_back(std::move(item));
  }
  // Lost before: when a PDDL+ plan ends, and its cost
  msg.has_end = plan.end.has_value();
  msg.end = plan.end.value_or(0.0);
  msg.has_cost = plan.cost.has_value();
  msg.cost = plan.cost.value_or(0.0);
  return msg;
}

}  // namespace

void KairosPlanSolver::configure(
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
  declare("anytime", rclcpp::ParameterValue(true));
  // 0.1 s: most of the anytime quality, without a half-second floor on every plan
  declare("no_improvement", rclcpp::ParameterValue(0.1));
  declare("search", rclcpp::ParameterValue(std::string("auto")));
  declare("engine", rclcpp::ParameterValue(std::string("auto")));
  declare("memory_limit_mb", rclcpp::ParameterValue(0));
  declare("threads", rclcpp::ParameterValue(1));
  declare("log_level", rclcpp::ParameterValue(std::string("warn")));
  declare("dump_dir", rclcpp::ParameterValue(std::string("")));
  declare("isolated", rclcpp::ParameterValue(false));
  declare("executable", rclcpp::ParameterValue(std::string("")));
}

std::optional<plansys2_msgs::msg::Plan> KairosPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace, const rclcpp::Duration solver_timeout)
{
  (void)node_namespace;
  cancel_requested_ = false;
  const auto logger = lc_node_->get_logger().get_child("kairos");

  // Leave time to hand the plan over before the planner node deadline
  using std::chrono::milliseconds;
  const auto node_ms = solver_timeout.to_chrono<milliseconds>();
  auto timeout = node_ms - std::max(kMinMargin, node_ms / 20);
  const double param_timeout = lc_node_->get_parameter(param("timeout")).as_double();
  if (param_timeout > 0.0) {
    timeout = std::min(timeout, milliseconds(static_cast<int64_t>(param_timeout * 1000.0)));
  }
  timeout = std::max(timeout, milliseconds(1));

  if (lc_node_->get_parameter(param("isolated")).as_bool()) {
    return plan_isolated(domain, problem, solver_timeout, timeout);
  }

  const auto parsed_domain = parse_domain(domain);
  if (!parsed_domain) {
    return std::nullopt;
  }
  auto parsed_problem = kairos::parse_problem(problem, *parsed_domain);
  if (!parsed_problem) {
    log_diagnostics(logger, parsed_problem.diagnostics(), problem);
    return std::nullopt;
  }

  kairos::SearchOptions options;
  options.timeout = timeout;

  options.anytime = lc_node_->get_parameter(param("anytime")).as_bool();
  const double no_improvement = lc_node_->get_parameter(param("no_improvement")).as_double();
  if (options.anytime && no_improvement > 0.0) {
    options.stop_after_no_improvement =
      milliseconds(static_cast<int64_t>(no_improvement * 1000.0));
  }

  options.search = lc_node_->get_parameter(param("search")).as_string();
  const auto search_presets = kairos::search_presets();
  if (options.search != "auto" &&
    std::find(search_presets.begin(), search_presets.end(), options.search) ==
    search_presets.end())
  {
    RCLCPP_WARN(logger, "Unknown search preset '%s', using auto", options.search.c_str());
    options.search = "auto";
  }

  const auto engine_name = lc_node_->get_parameter(param("engine")).as_string();
  if (auto e = engine(engine_name)) {
    options.engine = *e;
  } else {
    RCLCPP_WARN(logger, "Unknown engine '%s', using auto", engine_name.c_str());
  }

  options.memory_limit_mb = static_cast<size_t>(
    std::max<int64_t>(0, lc_node_->get_parameter(param("memory_limit_mb")).as_int()));
  options.threads = static_cast<unsigned>(
    std::max<int64_t>(1, lc_node_->get_parameter(param("threads")).as_int()));

  const auto level_name = lc_node_->get_parameter(param("log_level")).as_string();
  if (auto level = log_level(level_name)) {
    options.log.level = *level;
  } else {
    RCLCPP_WARN(logger, "Unknown log_level '%s', using warn", level_name.c_str());
    options.log.level = kairos::LogLevel::Warning;
  }
  options.log.callback = [logger](
    kairos::LogLevel level, kairos::LogChannel channel, std::string_view message) {
      const auto ch = kairos::to_string(channel);
      const int n = static_cast<int>(message.size());
      switch (level) {
        case kairos::LogLevel::Debug:
          RCLCPP_DEBUG(logger, "[%.*s] %.*s", static_cast<int>(ch.size()), ch.data(), n,
            message.data());
          break;
        case kairos::LogLevel::Info:
          RCLCPP_INFO(logger, "[%.*s] %.*s", static_cast<int>(ch.size()), ch.data(), n,
            message.data());
          break;
        case kairos::LogLevel::Warning:
          RCLCPP_WARN(logger, "[%.*s] %.*s", static_cast<int>(ch.size()), ch.data(), n,
            message.data());
          break;
        case kairos::LogLevel::Error:
          RCLCPP_ERROR(logger, "[%.*s] %.*s", static_cast<int>(ch.size()), ch.data(), n,
            message.data());
          break;
      }
    };

  const auto dump_dir = lc_node_->get_parameter(param("dump_dir")).as_string();
  if (!dump_dir.empty()) {
    options.dump_dir = std::filesystem::path(dump_dir);
  }

  kairos::Planner planner;
  auto search = planner.start(
    *parsed_problem, options, [logger](const kairos::PlanUpdate & update) {
      RCLCPP_DEBUG(
        logger, "Plan %u at %ld ms: %zu actions, makespan %g", update.index,
        static_cast<long>(update.elapsed.count()), update.plan.steps.size(),  // NOLINT
        update.plan.makespan());
    });
  while (search.running()) {
    if (cancel_requested_) {
      search.stop();
      break;
    }
    // Returns at once when a plan arrives or the search ends
    (void)search.next_plan(kCancelPoll);
  }
  const auto result = search.wait();

  log_diagnostics(logger, result.diagnostics);
  const auto status = kairos::to_string(result.status);
  const auto reason = kairos::to_string(result.stop_reason);
  if (!result.plan) {
    RCLCPP_WARN(
      logger, "No plan: %.*s (%.*s) after %.1f ms", static_cast<int>(status.size()),
      status.data(), static_cast<int>(reason.size()), reason.data(), result.stats.total_ms);
    return std::nullopt;
  }

  RCLCPP_INFO(
    logger, "Plan with %zu actions, makespan %g, first in %.1f ms, returned after %.1f ms (%.*s)",
    result.plan->steps.size(), result.plan->makespan(), result.stats.first_plan_ms.value_or(0.0),
    result.stats.total_ms, static_cast<int>(reason.size()), reason.data());
  return to_msg(*result.plan);
}

std::optional<plansys2_msgs::msg::Plan> KairosPlanSolver::plan_isolated(
  const std::string & domain, const std::string & problem,
  const rclcpp::Duration & solver_timeout, std::chrono::milliseconds timeout)
{
  const auto logger = lc_node_->get_logger().get_child("kairos");
  std::string folder = (std::filesystem::temp_directory_path() / "plansys2_kairos_XXXXXX").string();
  if (mkdtemp(folder.data()) == nullptr) {
    RCLCPP_ERROR(logger, "Cannot create a folder for Kairos: %s", std::strerror(errno));
    return std::nullopt;
  }
  const std::filesystem::path run(folder);
  const auto domain_path = run / "domain.pddl";
  const auto problem_path = run / "problem.pddl";
  const auto plan_path = run / "plan";

  std::optional<plansys2_msgs::msg::Plan> plan;
  if (!write_file(domain_path, domain) || !write_file(problem_path, problem)) {
    RCLCPP_ERROR(logger, "Cannot write the domain and problem to %s", folder.c_str());
  } else {
    auto executable = lc_node_->get_parameter(param("executable")).as_string();
    if (executable.empty()) {
      executable = std::filesystem::exists(KAIROS_EXECUTABLE) ? KAIROS_EXECUTABLE : "kairos";
    }
    std::string command = shell_quoted(executable) + " solve " +
      shell_quoted(domain_path.string()) + " " + shell_quoted(problem_path.string()) +
      " --format popf --timeout " +
      seconds(static_cast<double>(timeout.count()) / 1000.0);
    if (lc_node_->get_parameter(param("anytime")).as_bool()) {
      command += " --anytime";
      const double no_improvement = lc_node_->get_parameter(param("no_improvement")).as_double();
      if (no_improvement > 0.0) {
        command += " --no-improvement " + seconds(no_improvement);
      }
    } else {
      command += " --no-anytime";
    }
    command += " --search " + shell_quoted(lc_node_->get_parameter(param("search")).as_string());
    command += " --engine " + shell_quoted(lc_node_->get_parameter(param("engine")).as_string());
    const auto memory = lc_node_->get_parameter(param("memory_limit_mb")).as_int();
    if (memory > 0) {
      command += " --memory " + std::to_string(memory);
    }
    command += " --threads " +
      std::to_string(std::max<int64_t>(1, lc_node_->get_parameter(param("threads")).as_int()));

    // A crash or a hang of Kairos ends here, as an error of this plan
    if (execute_planner(command, solver_timeout, plan_path.string())) {
      std::ifstream in(plan_path);
      const std::string text(
        (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      if (text.find_first_not_of(" \t\r\n") == std::string::npos) {
        RCLCPP_WARN(logger, "No plan (isolated)");
      } else if (auto parsed = kairos::parse_plan(text)) {
        plan = to_msg(*parsed);
        RCLCPP_INFO(logger, "Plan with %zu actions (isolated)", plan->items.size());
      } else {
        log_diagnostics(logger, parsed.diagnostics(), text);
      }
    }
  }

  std::error_code error;
  std::filesystem::remove_all(run, error);
  return plan;
}

std::shared_ptr<const kairos::Domain> KairosPlanSolver::parse_domain(const std::string & domain)
{
  const size_t hash = std::hash<std::string>{}(domain);
  {
    std::lock_guard<std::mutex> lock(domain_mutex_);
    for (auto it = domains_.begin(); it != domains_.end(); ++it) {
      if (it->hash == hash && it->text == domain) {
        domains_.splice(domains_.begin(), domains_, it);
        return it->domain;
      }
    }
  }

  // Parsed outside the lock: other calls with cached domains do not wait
  auto parsed = kairos::parse_domain(domain);
  if (!parsed) {
    log_diagnostics(lc_node_->get_logger().get_child("kairos"), parsed.diagnostics(), domain);
    return nullptr;
  }
  auto result = std::make_shared<const kairos::Domain>(std::move(parsed).value());

  std::lock_guard<std::mutex> lock(domain_mutex_);
  domains_.push_front(CachedDomain{hash, domain, result});
  if (domains_.size() > kDomainCacheSize) {
    domains_.pop_back();
  }
  return result;
}

std::set<std::string> KairosPlanSolver::supported_requirements() const
{
  return known_pddl_requirements();
}

bool KairosPlanSolver::isDomainValid(
  const std::string & domain, const std::string & node_namespace)
{
  (void)node_namespace;
  const auto report = kairos::check_domain(domain);
  if (!report.plannable) {
    log_diagnostics(lc_node_->get_logger().get_child("kairos"), report.diagnostics, domain);
  }
  return report.plannable;
}

}  // namespace plansys2

PLUGINLIB_EXPORT_CLASS(plansys2::KairosPlanSolver, plansys2::PlanSolverBase);
