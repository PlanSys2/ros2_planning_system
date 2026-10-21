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

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/Utils.hpp"
#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

ProblemExpertClient::ProblemExpertClient(ClientNode::Interfaces node)
: node_(std::make_shared<ClientNode>(node)),
  active_(std::make_shared<ActiveComponent>(node_, "problem_expert"))
{
  namespace srv = plansys2_msgs::srv;
  set_continuous_effect_client_ = std::make_shared<ServiceClient<srv::SetContinuousEffect>>(
    node_, "problem_expert/set_continuous_effect", active_);
  add_problem_client_ =
    std::make_shared<ServiceClient<srv::AddProblem>>(node_, "problem_expert/add_problem", active_);
  add_problem_goal_client_ = std::make_shared<ServiceClient<srv::AddProblemGoal>>(
    node_, "problem_expert/add_problem_goal", active_);
  add_problem_instance_client_ = std::make_shared<ServiceClient<srv::AffectParam>>(
    node_, "problem_expert/add_problem_instance", active_);
  add_problem_predicate_client_ = std::make_shared<ServiceClient<srv::AffectNode>>(
    node_, "problem_expert/add_problem_predicate", active_);
  add_problem_function_client_ = std::make_shared<ServiceClient<srv::AffectNode>>(
    node_, "problem_expert/add_problem_function", active_);
  get_problem_goal_client_ = std::make_shared<ServiceClient<srv::GetProblemGoal>>(
    node_, "problem_expert/get_problem_goal", active_);
  get_problem_instance_details_client_ =
    std::make_shared<ServiceClient<srv::GetProblemInstanceDetails>>(
    node_, "problem_expert/get_problem_instance", active_);
  get_problem_instances_client_ = std::make_shared<ServiceClient<srv::GetProblemInstances>>(
    node_, "problem_expert/get_problem_instances", active_);
  get_problem_predicate_details_client_ = std::make_shared<ServiceClient<srv::GetNodeDetails>>(
    node_, "problem_expert/get_problem_predicate", active_);
  get_problem_predicates_client_ = std::make_shared<ServiceClient<srv::GetStates>>(
    node_, "problem_expert/get_problem_predicates", active_);
  get_problem_function_details_client_ = std::make_shared<ServiceClient<srv::GetNodeDetails>>(
    node_, "problem_expert/get_problem_function", active_);
  get_problem_functions_client_ = std::make_shared<ServiceClient<srv::GetStates>>(
    node_, "problem_expert/get_problem_functions", active_);
  get_problem_client_ =
    std::make_shared<ServiceClient<srv::GetProblem>>(node_, "problem_expert/get_problem", active_);
  remove_problem_goal_client_ = std::make_shared<ServiceClient<srv::RemoveProblemGoal>>(
    node_, "problem_expert/remove_problem_goal", active_);
  set_problem_metric_client_ = std::make_shared<ServiceClient<srv::SetProblemMetric>>(
    node_, "problem_expert/set_problem_metric", active_);
  get_problem_metric_client_ = std::make_shared<ServiceClient<srv::GetProblemMetric>>(
    node_, "problem_expert/get_problem_metric", active_);
  remove_problem_metric_client_ = std::make_shared<ServiceClient<srv::RemoveProblemMetric>>(
    node_, "problem_expert/remove_problem_metric", active_);
  clear_problem_knowledge_client_ = std::make_shared<ServiceClient<srv::ClearProblemKnowledge>>(
    node_, "problem_expert/clear_problem_knowledge", active_);
  remove_problem_instance_client_ = std::make_shared<ServiceClient<srv::AffectParam>>(
    node_, "problem_expert/remove_problem_instance", active_);
  remove_problem_predicate_client_ = std::make_shared<ServiceClient<srv::AffectNode>>(
    node_, "problem_expert/remove_problem_predicate", active_);
  remove_problem_function_client_ = std::make_shared<ServiceClient<srv::AffectNode>>(
    node_, "problem_expert/remove_problem_function", active_);
  exist_problem_predicate_client_ = std::make_shared<ServiceClient<srv::ExistNode>>(
    node_, "problem_expert/exist_problem_predicate", active_);
  exist_problem_function_client_ = std::make_shared<ServiceClient<srv::ExistNode>>(
    node_, "problem_expert/exist_problem_function", active_);
  update_problem_function_client_ = std::make_shared<ServiceClient<srv::AffectNode>>(
    node_, "problem_expert/update_problem_function", active_);
  is_problem_goal_satisfied_client_ = std::make_shared<ServiceClient<srv::IsProblemGoalSatisfied>>(
    node_, "problem_expert/is_problem_goal_satisfied", active_);

  problem_sub_ = node_->create_subscription<plansys2_msgs::msg::Problem>(
    "problem_expert/problem",
    rclcpp::QoS(100), [this](plansys2_msgs::msg::Problem::SharedPtr msg) {
      // Published before this client's last change: it would bring the cache back (#438)
      if (rclcpp::Time(msg->stamp, cache_valid_since_.get_clock_type()) < cache_valid_since_) {
        return;
      }
      cached_problem_ = msg->problem;
      problem_ts_ = msg->stamp;
    });

  update_problem_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    "problem_expert/update_notify", 100,
    [this](std_msgs::msg::Empty::SharedPtr msg){
      (void)msg;
      update_time_ = this->node_->now();
    });

  problem_ts_ = node_->now();
  // Same clock as the times it is compared with
  update_time_ = node_->now();
}

template<typename ServiceT>
bool
ProblemExpertClient::call_change(
  typename ServiceClient<ServiceT>::SharedPtr client,
  typename ServiceT::Request::SharedPtr request)
{
  invalidate_cache();

  auto result = client->call(request);
  if (!result) {
    return false;
  }

  if (result->success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      client->name() << ": " << result->error_info);
    return false;
  }
}

std::vector<plansys2::Instance>
ProblemExpertClient::getInstances()
{
  return tryGetInstances().value_or(std::vector<plansys2::Instance>{});
}

std::optional<std::vector<plansys2::Instance>>
ProblemExpertClient::tryGetInstances()
{
  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstances::Request>();

  auto result = get_problem_instances_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    return plansys2::convertVector<plansys2::Instance, plansys2_msgs::msg::Param>(
      result->instances);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      node_->get_namespace() <<
        get_problem_instances_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addInstance(const plansys2::Instance & instance)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  request->param = instance;

  return call_change<plansys2_msgs::srv::AffectParam>(add_problem_instance_client_, request);
}

bool
ProblemExpertClient::removeInstance(const plansys2::Instance & instance)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  request->param = instance;

  return call_change<plansys2_msgs::srv::AffectParam>(remove_problem_instance_client_, request);
}


std::optional<plansys2::Instance>
ProblemExpertClient::getInstance(const std::string & name)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstanceDetails::Request>();

  request->instance = name;

  auto result = get_problem_instance_details_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    return result->instance;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instance_details_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}

std::vector<plansys2::Predicate>
ProblemExpertClient::getPredicates()
{
  return tryGetPredicates().value_or(std::vector<plansys2::Predicate>{});
}

std::optional<std::vector<plansys2::Predicate>>
ProblemExpertClient::tryGetPredicates()
{
  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto result = get_problem_predicates_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    return plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(
      result->states);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addPredicate(const plansys2::Predicate & predicate)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = predicate;

  return call_change<plansys2_msgs::srv::AffectNode>(add_problem_predicate_client_, request);
}

bool
ProblemExpertClient::removePredicate(const plansys2::Predicate & predicate)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = predicate;

  return call_change<plansys2_msgs::srv::AffectNode>(remove_problem_predicate_client_, request);
}

bool
ProblemExpertClient::existPredicate(const plansys2::Predicate & predicate)
{
  return tryExistPredicate(predicate).value_or(false);
}

std::optional<bool>
ProblemExpertClient::tryExistPredicate(const plansys2::Predicate & predicate)
{
  auto request = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  request->node = predicate;

  auto result = exist_problem_predicate_client_->call(request);
  if (!result) {
    return {};
  }

  return result->exist;
}

std::optional<plansys2::Predicate>
ProblemExpertClient::getPredicate(const std::string & predicate)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = predicate;

  auto result = get_problem_predicate_details_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    return result->node;
  } else {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}

std::vector<plansys2::Function>
ProblemExpertClient::getFunctions()
{
  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto result = get_problem_functions_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    return plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
      result->states);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addFunction(const plansys2::Function & function)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  return call_change<plansys2_msgs::srv::AffectNode>(add_problem_function_client_, request);
}

bool
ProblemExpertClient::removeFunction(const plansys2::Function & function)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  return call_change<plansys2_msgs::srv::AffectNode>(remove_problem_function_client_, request);
}

bool
ProblemExpertClient::existFunction(const plansys2::Function & function)
{
  return tryExistFunction(function).value_or(false);
}

std::optional<bool>
ProblemExpertClient::tryExistFunction(const plansys2::Function & function)
{
  auto request = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  request->node = function;

  auto result = exist_problem_function_client_->call(request);
  if (!result) {
    return {};
  }

  return result->exist;
}

bool ProblemExpertClient::updateFunction(const plansys2::Function & function)
{
  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  return call_change<plansys2_msgs::srv::AffectNode>(update_problem_function_client_, request);
}

std::optional<plansys2::Function>
ProblemExpertClient::getFunction(const std::string & function)
{
  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = function;

  auto result = get_problem_function_details_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    return result->node;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}


plansys2::Goal
ProblemExpertClient::getGoal()
{
  plansys2_msgs::msg::Tree ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemGoal::Request>();

  auto result = get_problem_goal_client_->call(request);
  if (!result) {
    return ret;
  }

  if (result->success) {
    return result->tree;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_goal_client_->name() << ": " <<
        result->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::setGoal(const plansys2::Goal & goal)
{
  auto request = std::make_shared<plansys2_msgs::srv::AddProblemGoal::Request>();
  request->tree = goal;

  return call_change<plansys2_msgs::srv::AddProblemGoal>(add_problem_goal_client_, request);
}

bool
ProblemExpertClient::isGoalSatisfied(const plansys2::Goal & goal)
{
  auto request = std::make_shared<plansys2_msgs::srv::IsProblemGoalSatisfied::Request>();
  request->tree = goal;

  auto result = is_problem_goal_satisfied_client_->call(request);
  if (!result) {
    return false;
  }

  if (result->success) {
    return result->satisfied;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      is_problem_goal_satisfied_client_->name() << ": " <<
        result->error_info);
    return false;
  }
}

bool
ProblemExpertClient::clearGoal()
{
  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemGoal::Request>();

  return call_change<plansys2_msgs::srv::RemoveProblemGoal>(remove_problem_goal_client_, request);
}

plansys2::Metric
ProblemExpertClient::getMetric()
{
  plansys2::Metric ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemMetric::Request>();

  auto result = get_problem_metric_client_->call(request);
  if (!result) {
    return ret;
  }

  if (result->success) {
    return result->metric;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_metric_client_->name() << ": " <<
        result->error_info);
  }

  return ret;
}

bool
ProblemExpertClient::setMetric(const plansys2::Metric & metric)
{
  auto request = std::make_shared<plansys2_msgs::srv::SetProblemMetric::Request>();
  request->metric = metric;

  return call_change<plansys2_msgs::srv::SetProblemMetric>(set_problem_metric_client_, request);
}

bool
ProblemExpertClient::clearMetric()
{
  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemMetric::Request>();

  return call_change<plansys2_msgs::srv::RemoveProblemMetric>(
    remove_problem_metric_client_, request);
}

bool
ProblemExpertClient::clearKnowledge()
{
  auto request = std::make_shared<plansys2_msgs::srv::ClearProblemKnowledge::Request>();

  return call_change<plansys2_msgs::srv::ClearProblemKnowledge>(
    clear_problem_knowledge_client_, request);
}

void
ProblemExpertClient::invalidate_cache()
{
  cached_problem_.clear();
  cache_valid_since_ = node_->now();
}

std::tuple<std::string, rclcpp::Time>
ProblemExpertClient::getProblemWithTimestamp(bool use_cache)
{
  return {getProblem(use_cache), problem_ts_};
}

std::string
ProblemExpertClient::getProblem(bool use_cache)
{
  node_->serve();  // The problem published meanwhile
  if (use_cache && cached_problem_ != "") {
    return cached_problem_;
  } else {
    return getProblem();
  }
}

std::string
ProblemExpertClient::getProblem()
{
  auto request = std::make_shared<plansys2_msgs::srv::GetProblem::Request>();

  auto result = get_problem_client_->call(request);
  if (!result) {
    return {};
  }

  if (result->success) {
    cached_problem_ = result->problem;
    problem_ts_ = result->stamp;

    return result->problem;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_client_->name() << ": " <<
        result->error_info);
    return {};
  }
}

bool
ProblemExpertClient::addProblem(const std::string & problem_str)
{
  auto request = std::make_shared<plansys2_msgs::srv::AddProblem::Request>();
  request->problem = problem_str;

  return call_change<plansys2_msgs::srv::AddProblem>(add_problem_client_, request);
}

bool
ProblemExpertClient::setContinuousEffect(
  const std::string & id, const plansys2_msgs::msg::Tree & effects, bool active)
{
  auto request = std::make_shared<plansys2_msgs::srv::SetContinuousEffect::Request>();
  request->id = id;
  request->effects = effects;
  request->active = active;
  return call_change<plansys2_msgs::srv::SetContinuousEffect>(
    set_continuous_effect_client_, request);
}

}  // namespace plansys2
