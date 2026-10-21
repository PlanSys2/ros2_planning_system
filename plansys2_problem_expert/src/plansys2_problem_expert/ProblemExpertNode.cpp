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

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

std::vector<std::string> tokenize(const std::string & string, const std::string & delim)
{
  std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
  std::vector<std::string> tokens;

  while (lastPos != std::string::npos) {
    if (pos != lastPos) {
      tokens.push_back(string.substr(lastPos, pos - lastPos));
    }
    lastPos = pos;
    if (lastPos == std::string::npos || lastPos + 1 == string.length()) {
      break;
    }
    pos = string.find_first_of(delim, ++lastPos);
  }

  return tokens;
}

using namespace std::chrono_literals;  // NOLINT

namespace plansys2
{

ProblemExpertNode::ProblemExpertNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("problem_expert", options)
{
  declare_parameter("model_file", "");
  declare_parameter("problem_file", "");
  // Where the problem is kept while the node runs, to come back with it if it restarts
  declare_parameter("state_file", "");
  // PDDL+: the processes, events and continuous effects of running actions act on the problem,
  // except on the functions perception measures
  declare_parameter("simulate_dynamics", true);
  declare_parameter("measured_functions", std::vector<std::string>{});

  set_continuous_effect_service_ = create_service<plansys2_msgs::srv::SetContinuousEffect>(
    "problem_expert/set_continuous_effect",
    guarded<plansys2_msgs::srv::SetContinuousEffect>(
      get_logger(),
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<plansys2_msgs::srv::SetContinuousEffect::Request> request,
        const std::shared_ptr<plansys2_msgs::srv::SetContinuousEffect::Response> response) {
        if (problem_expert_ == nullptr) {
          response->success = false;
          response->error_info = plansys2::kNotActive;
          return;
        }
        response->success = problem_expert_->setContinuousEffect(
        request->id, request->effects, request->active);
        if (!response->success) {
          response->error_info = "Continuous effects not valid";
        }
    }));

  add_problem_service_ = create_service<plansys2_msgs::srv::AddProblem>(
    "problem_expert/add_problem",
    guarded<plansys2_msgs::srv::AddProblem>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::add_problem_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  add_problem_goal_service_ = create_service<plansys2_msgs::srv::AddProblemGoal>(
    "problem_expert/add_problem_goal",
    guarded<plansys2_msgs::srv::AddProblemGoal>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::add_problem_goal_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  add_problem_instance_service_ = create_service<plansys2_msgs::srv::AffectParam>(
    "problem_expert/add_problem_instance",
    guarded<plansys2_msgs::srv::AffectParam>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::add_problem_instance_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  add_problem_predicate_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_predicate",
    guarded<plansys2_msgs::srv::AffectNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::add_problem_predicate_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  add_problem_function_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_function",
    guarded<plansys2_msgs::srv::AffectNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::add_problem_function_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_goal_service_ = create_service<plansys2_msgs::srv::GetProblemGoal>(
    "problem_expert/get_problem_goal",
    guarded<plansys2_msgs::srv::GetProblemGoal>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::get_problem_goal_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_instance_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "problem_expert/get_problem_instance",
    guarded<plansys2_msgs::srv::GetProblemInstanceDetails>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::get_problem_instance_details_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_instances_service_ = create_service<plansys2_msgs::srv::GetProblemInstances>(
    "problem_expert/get_problem_instances",
    guarded<plansys2_msgs::srv::GetProblemInstances>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::get_problem_instances_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_predicate",
    guarded<plansys2_msgs::srv::GetNodeDetails>(get_logger(), std::bind(
      &ProblemExpertNode::get_problem_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));

  get_problem_predicates_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_predicates",
    guarded<plansys2_msgs::srv::GetStates>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::get_problem_predicates_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_function_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_function",
    guarded<plansys2_msgs::srv::GetNodeDetails>(get_logger(), std::bind(
      &ProblemExpertNode::get_problem_function_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));

  get_problem_functions_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_functions",
    guarded<plansys2_msgs::srv::GetStates>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::get_problem_functions_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_service_ = create_service<plansys2_msgs::srv::GetProblem>(
    "problem_expert/get_problem",
    guarded<plansys2_msgs::srv::GetProblem>(get_logger(), std::bind(
      &ProblemExpertNode::get_problem_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));

  is_problem_goal_satisfied_service_ = create_service<plansys2_msgs::srv::IsProblemGoalSatisfied>(
    "problem_expert/is_problem_goal_satisfied",
    guarded<plansys2_msgs::srv::IsProblemGoalSatisfied>(get_logger(), std::bind(
      &ProblemExpertNode::is_problem_goal_satisfied_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));

  remove_problem_goal_service_ = create_service<plansys2_msgs::srv::RemoveProblemGoal>(
    "problem_expert/remove_problem_goal",
    guarded<plansys2_msgs::srv::RemoveProblemGoal>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::remove_problem_goal_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  set_problem_metric_service_ = create_service<plansys2_msgs::srv::SetProblemMetric>(
    "problem_expert/set_problem_metric",
    guarded<plansys2_msgs::srv::SetProblemMetric>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::set_problem_metric_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_problem_metric_service_ = create_service<plansys2_msgs::srv::GetProblemMetric>(
    "problem_expert/get_problem_metric",
    guarded<plansys2_msgs::srv::GetProblemMetric>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::get_problem_metric_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  remove_problem_metric_service_ = create_service<plansys2_msgs::srv::RemoveProblemMetric>(
    "problem_expert/remove_problem_metric",
    guarded<plansys2_msgs::srv::RemoveProblemMetric>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::remove_problem_metric_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  clear_problem_knowledge_service_ = create_service<plansys2_msgs::srv::ClearProblemKnowledge>(
    "problem_expert/clear_problem_knowledge",
    guarded<plansys2_msgs::srv::ClearProblemKnowledge>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::clear_problem_knowledge_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  remove_problem_instance_service_ = create_service<plansys2_msgs::srv::AffectParam>(
    "problem_expert/remove_problem_instance",
    guarded<plansys2_msgs::srv::AffectParam>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::remove_problem_instance_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  remove_problem_predicate_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_predicate",
    guarded<plansys2_msgs::srv::AffectNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::remove_problem_predicate_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  remove_problem_function_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_function",
    guarded<plansys2_msgs::srv::AffectNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::remove_problem_function_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  exist_problem_predicate_service_ = create_service<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_predicate",
    guarded<plansys2_msgs::srv::ExistNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::exist_problem_predicate_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  exist_problem_function_service_ = create_service<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_function",
    guarded<plansys2_msgs::srv::ExistNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::exist_problem_function_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  update_problem_function_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/update_problem_function",
    guarded<plansys2_msgs::srv::AffectNode>(
      get_logger(),
      std::bind(
        &ProblemExpertNode::update_problem_function_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  problem_pub_ = create_publisher<plansys2_msgs::msg::Problem>(
    "problem_expert/problem",
    rclcpp::QoS(100));

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "problem_expert/update_notify",
    rclcpp::QoS(100));

  knowledge_pub_ = create_publisher<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge",
    rclcpp::QoS(100).transient_local());

  domain_sub_ = create_subscription<std_msgs::msg::String>(
    "domain_expert/domain",
    rclcpp::QoS(100).transient_local(),
    std::bind(&ProblemExpertNode::domain_topic_callback, this, std::placeholders::_1));

  // The domain expert asks here before it takes a new domain: it answers once this one did
  update_domain_service_ = create_service<plansys2_msgs::srv::ChangeDomain>(
    "problem_expert/update_domain",
    guarded<plansys2_msgs::srv::ChangeDomain>(
      get_logger(),
    std::bind(
      &ProblemExpertNode::update_domain_service_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3)));
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ProblemExpertNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  // (fmrico) Here we could have a discussion if we should read the domain from file or
  // from the domain_expert service. Then, we should configure first domain_expert node
  auto model_file = get_parameter("model_file").get_value<std::string>();

  auto model_files = tokenize(model_file, ":");

  // Every domain file must be there and readable
  std::vector<std::string> domains;
  for (const auto & file : model_files) {
    std::ifstream domain_ifs(file);
    if (file.empty() || !domain_ifs) {
      RCLCPP_ERROR(get_logger(), "Cannot read model_file [%s]", file.c_str());
      return CallbackReturnT::FAILURE;
    }
    domains.emplace_back(
      (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());
  }
  if (domains.empty()) {
    RCLCPP_ERROR(get_logger(), "No model_file given");
    return CallbackReturnT::FAILURE;
  }

  auto domain_expert = std::make_shared<DomainExpert>(domains[0]);

  for (size_t i = 1; i < domains.size(); i++) {
    domain_expert->extendDomain(domains[i]);
  }
  // A domain read in part would have no actions: as the domain expert, do not start
  if (!domain_expert->getParseError().empty()) {
    RCLCPP_ERROR(
      get_logger(), "PlanSys2 cannot read this domain: %s", domain_expert->getParseError().c_str());
    return CallbackReturnT::FAILURE;
  }

  problem_expert_ = std::make_shared<ProblemExpert>(domain_expert);
  // Timed literals and fluents come by the time of the node, simulated or not
  problem_expert_->setClock([this]() {return now().seconds();});
  {
    const auto measured = get_parameter("measured_functions").as_string_array();
    problem_expert_->setMeasuredFunctions(std::set<std::string>(measured.begin(), measured.end()));
  }

  state_file_ = get_parameter("state_file").get_value<std::string>();
  if (!state_file_.empty() && state_file_[0] == '~') {
    if (const char * home = std::getenv("HOME")) {
      state_file_.replace(0, 1, home);
    }
  }
  const bool recovered = load_state();

  auto problem_file = get_parameter("problem_file").get_value<std::string>();
  if (!recovered && !problem_file.empty()) {
    std::ifstream problem_ifs(problem_file);
    if (!problem_ifs) {
      RCLCPP_ERROR(get_logger(), "Cannot read problem_file %s", problem_file.c_str());
      return CallbackReturnT::FAILURE;
    }
    std::string problem_str((
        std::istreambuf_iterator<char>(problem_ifs)),
      std::istreambuf_iterator<char>());
    if (!problem_expert_->addProblem(problem_str)) {
      RCLCPP_ERROR(get_logger(), "Cannot load problem_file %s", problem_file.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  // The domain expert changed its domain before we were configured: take its domain
  if (!last_domain_.empty() && last_domain_ != problem_expert_->getDomain()) {
    if (problem_expert_->updateDomain(last_domain_)) {
      RCLCPP_INFO(get_logger(), "[%s] Took the current domain of the domain expert", get_name());
    } else {
      RCLCPP_ERROR(get_logger(), "[%s] The current domain could not be loaded", get_name());
    }
  }

  // Saved at most every 100 ms: a burst of changes writes it once
  if (!state_file_.empty()) {
    state_timer_ = create_wall_timer(
      100ms, [this]() {
        if (state_dirty_.exchange(false)) {
          save_state();
        }
      });
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

void
ProblemExpertNode::notify_change()
{
  update_pub_->publish(std_msgs::msg::Empty());
  knowledge_pub_->publish(*get_knowledge_as_msg());

  plansys2_msgs::msg::Problem problem_msg;
  problem_msg.problem = problem_expert_->getProblem();
  problem_msg.stamp = now();
  problem_pub_->publish(problem_msg);
  state_dirty_ = true;
}

void
ProblemExpertNode::save_state()
{
  auto expert = problem_expert_;
  if (state_file_.empty() || !expert) {
    return;
  }
  // Written aside and renamed: a crash while writing leaves the last whole one
  const std::filesystem::path path(state_file_);
  const auto tmp = path.string() + ".tmp";
  std::error_code error;
  std::filesystem::create_directories(path.parent_path(), error);
  {
    std::ofstream out(tmp, std::ios::trunc);
    out << expert->getProblem();
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Cannot write the state to %s", tmp.c_str());
      return;
    }
  }
  std::filesystem::rename(tmp, path, error);
  if (error) {
    RCLCPP_ERROR(
      get_logger(), "Cannot write the state to %s: %s", state_file_.c_str(),
      error.message().c_str());
  }
}

bool
ProblemExpertNode::load_state()
{
  if (state_file_.empty() || !std::filesystem::exists(state_file_)) {
    return false;
  }
  std::ifstream in(state_file_);
  const std::string problem((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (problem.empty() || !problem_expert_->addProblem(problem)) {
    RCLCPP_ERROR(
      get_logger(), "[%s] The state in %s cannot be read: starting without it", get_name(),
      state_file_.c_str());
    problem_expert_->clearKnowledge();
    return false;
  }
  RCLCPP_WARN(
    get_logger(), "[%s] Came back with the problem it had, from %s", get_name(),
    state_file_.c_str());
  return true;
}

void
ProblemExpertNode::remove_state()
{
  state_timer_.reset();
  state_dirty_ = false;
  if (!state_file_.empty()) {
    std::error_code error;
    std::filesystem::remove(state_file_, error);
  }
}

CallbackReturnT
ProblemExpertNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  update_pub_->on_activate();
  knowledge_pub_->on_activate();
  problem_pub_->on_activate();

  // Timed facts are applied when their time comes, and the dynamics act, as any other change
  last_step_ = now();
  timed_facts_timer_ = create_wall_timer(
    50ms, [this]() {
      if (!problem_expert_) {
        return;
      }
      const auto stamp = now();
      const double dt = std::max(0.0, (stamp - last_step_).seconds());
      last_step_ = stamp;
      bool changed = problem_expert_->applyTimedFacts();
      if (get_parameter("simulate_dynamics").as_bool()) {
        changed = problem_expert_->stepDynamics(dt) || changed;
      }
      // What happens by itself, for who follows the execution
      for (const auto & happened : problem_expert_->takeHappened()) {
        RCLCPP_INFO(get_logger(), "[dynamics] %s", happened.c_str());
      }
      if (changed) {
        notify_change();
      }
    });
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  timed_facts_timer_.reset();
  update_pub_->on_deactivate();
  knowledge_pub_->on_deactivate();
  problem_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  // Unconfigured: the knowledge goes, and the services answer that they are not active; it is
  // not kept either: cleaned up on purpose, it starts from nothing
  remove_state();
  problem_expert_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  remove_state();  // Ended on purpose: a new start does not take it
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
ProblemExpertNode::add_problem_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblem::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblem::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    RCLCPP_INFO(get_logger(), "Adding problem:\n%s", request->problem.c_str());
    response->success = problem_expert_->addProblem(request->problem);

    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Problem not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    if (!parser::pddl::empty(request->tree)) {
      response->success = problem_expert_->setGoal(request->tree);
      if (response->success) {
        notify_change();
      } else {
        response->error_info = "Goal not valid";
      }
    } else {
      response->success = false;
      response->error_info = "Malformed expression";
    }
  }
}

void
ProblemExpertNode::add_problem_instance_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->addInstance(request->param);
    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Instance not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->addPredicate(request->node);
    if (response->success) {
      notify_change();
    } else {
      response->error_info =
        "Predicate [" + parser::pddl::toString(request->node) + "] not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->addFunction(request->node);
    if (response->success) {
      notify_change();
    } else {
      response->error_info =
        "Function [" + parser::pddl::toString(request->node) + "] not valid";
    }
  }
}

void
ProblemExpertNode::get_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response)
{
  (void)request_header;
  (void)request;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->tree = problem_expert_->getGoal();
  }
}

void
ProblemExpertNode::get_problem_instance_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    auto instance = problem_expert_->getInstance(request->instance);
    if (instance) {
      response->success = true;
      response->instance = instance.value();
    } else {
      response->success = false;
      response->error_info = "Instance not found";
    }
  }
}

void
ProblemExpertNode::get_problem_instances_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->instances = plansys2::convertVector<plansys2_msgs::msg::Param, plansys2::Instance>(
      problem_expert_->getInstances());
  }
}

void
ProblemExpertNode::get_problem_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    auto predicate = problem_expert_->getPredicate(request->expression);
    if (predicate) {
      response->node = predicate.value();
      response->success = true;
    } else {
      response->success = false;
      response->error_info = "Predicate not found";
    }
  }
}

void
ProblemExpertNode::get_problem_predicates_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
      problem_expert_->getPredicates());
  }
}

void
ProblemExpertNode::get_problem_function_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    auto function = problem_expert_->getFunction(request->expression);
    if (function) {
      response->node = function.value();
      response->success = true;
    } else {
      response->success = false;
      response->error_info = "Function not found";
    }
  }
}

void
ProblemExpertNode::get_problem_functions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Function>(
      problem_expert_->getFunctions());
  }
}

void
ProblemExpertNode::get_problem_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->stamp = now();
    response->problem = problem_expert_->getProblem();
  }
}

void
ProblemExpertNode::is_problem_goal_satisfied_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->satisfied = problem_expert_->isGoalSatisfied(request->tree);
  }
}

void
ProblemExpertNode::remove_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->clearGoal();

    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Error clearing goal";
    }
  }
}

void
ProblemExpertNode::set_problem_metric_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::SetProblemMetric::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::SetProblemMetric::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->setMetric(request->metric);
    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Metric not valid";
    }
  }
}

void
ProblemExpertNode::get_problem_metric_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemMetric::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemMetric::Response> response)
{
  (void)request_header;
  (void)request;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->metric = problem_expert_->getMetric();
  }
}

void
ProblemExpertNode::remove_problem_metric_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemMetric::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemMetric::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->clearMetric();

    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Error clearing metric";
    }
  }
}

void
ProblemExpertNode::clear_problem_knowledge_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Response> response)
{
  (void)request;
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->clearKnowledge();

    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Error clearing knowledge";
    }
  }
}

bool
ProblemExpertNode::applyNewDomain(const std::string & domain)
{
  if (domain == problem_expert_->getDomain()) {
    return true;
  }
  if (!problem_expert_->updateDomain(domain)) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[" << get_name() << "] Failed to update domain: the new domain could not "
        "be loaded, problem knowledge left unchanged");
    return false;
  }

  notify_change();

  RCLCPP_INFO(get_logger(), "[%s] Reconciled problem knowledge with new domain", get_name());
  return true;
}

void
ProblemExpertNode::update_domain_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ChangeDomain::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ChangeDomain::Response> response)
{
  (void)request_header;
  last_domain_ = request->domain;
  if (problem_expert_ == nullptr) {
    // Taken when configured
    response->success = true;
    return;
  }
  response->success = applyNewDomain(request->domain);
  if (!response->success) {
    response->error_info = "The problem expert could not load the domain";
  }
}

void
ProblemExpertNode::domain_topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_domain_ = msg->data;
  // Not configured yet: on_configure takes it if it is not the one of model_file
  if (problem_expert_ == nullptr) {
    return;
  }

  // By content: the replay of the domain we already have changes nothing, a newer one does
  if (msg->data == problem_expert_->getDomain()) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "[%s] Domain changed, reconciling problem knowledge...", get_name());
  applyNewDomain(msg->data);
}

void
ProblemExpertNode::remove_problem_instance_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->removeInstance(request->param);
    if (response->success) {
      notify_change();

    } else {
      response->error_info = "Error removing instance";
    }
  }
}

void
ProblemExpertNode::remove_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->removePredicate(request->node);
    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Error removing predicate";
    }
  }
}

void
ProblemExpertNode::remove_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->removeFunction(request->node);
    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Error removing function";
    }
  }
}

void
ProblemExpertNode::exist_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->exist = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->exist = problem_expert_->existPredicate(request->node);
  }
}

void
ProblemExpertNode::exist_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->exist = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->exist = problem_expert_->existFunction(request->node);
  }
}

void
ProblemExpertNode::update_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  (void)request_header;
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = problem_expert_->updateFunction(request->node);
    if (response->success) {
      notify_change();
    } else {
      response->error_info = "Function not valid";
    }
  }
}

plansys2_msgs::msg::Knowledge::SharedPtr
ProblemExpertNode::get_knowledge_as_msg() const
{
  auto ret_msgs = std::make_shared<plansys2_msgs::msg::Knowledge>();

  for (const auto & instance : problem_expert_->getInstances()) {
    ret_msgs->instances.push_back(instance.name);
  }

  for (const auto & predicate : problem_expert_->getPredicates()) {
    ret_msgs->predicates.push_back(parser::pddl::toString(predicate));
  }

  // With its value, as in :init: a number, or the object of an object fluent
  for (const auto & function : problem_expert_->getFunctions()) {
    std::string value = function.object;
    if (value.empty()) {
      value = parser::pddl::numberToString(function.value);
      if (value.find('.') != std::string::npos) {
        value.erase(value.find_last_not_of('0') + 1);
        if (value.back() == '.') {value.pop_back();}
      }
    }
    ret_msgs->functions.push_back("(= " + parser::pddl::toString(function) + " " + value + ")");
  }

  auto goal = problem_expert_->getGoal();
  ret_msgs->goal = parser::pddl::toString(goal);
  ret_msgs->metric = parser::pddl::toString(problem_expert_->getMetric());
  ret_msgs->constraints = parser::pddl::toString(problem_expert_->getConstraints());

  return ret_msgs;
}

}  // namespace plansys2
