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

#include <string>
#include <memory>
#include <vector>

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

namespace plansys2
{

ProblemExpertNode::ProblemExpertNode()
: rclcpp_lifecycle::LifecycleNode("problem_expert")
{
  declare_parameter("model_file", "");

  add_problem_goal_service_ = create_service<plansys2_msgs::srv::AddProblemGoal>(
    "problem_expert/add_problem_goal",
    std::bind(
      &ProblemExpertNode::add_problem_goal_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_instance_service_ = create_service<plansys2_msgs::srv::AddProblemInstance>(
    "problem_expert/add_problem_instance",
    std::bind(
      &ProblemExpertNode::add_problem_instance_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_predicate_service_ = create_service<plansys2_msgs::srv::AddProblemPredicate>(
    "problem_expert/add_problem_predicate",
    std::bind(
      &ProblemExpertNode::add_problem_predicate_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_function_service_ = create_service<plansys2_msgs::srv::AddProblemFunction>(
    "problem_expert/add_problem_function",
    std::bind(
      &ProblemExpertNode::add_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_goal_service_ = create_service<plansys2_msgs::srv::GetProblemGoal>(
    "problem_expert/get_problem_goal",
    std::bind(
      &ProblemExpertNode::get_problem_goal_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_instance_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "problem_expert/get_problem_instance",
    std::bind(
      &ProblemExpertNode::get_problem_instance_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_instances_service_ = create_service<plansys2_msgs::srv::GetProblemInstances>(
    "problem_expert/get_problem_instances",
    std::bind(
      &ProblemExpertNode::get_problem_instances_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemPredicateDetails>(
    "problem_expert/get_problem_predicate", std::bind(
      &ProblemExpertNode::get_problem_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_predicates_service_ = create_service<plansys2_msgs::srv::GetProblemPredicates>(
    "problem_expert/get_problem_predicates",
    std::bind(
      &ProblemExpertNode::get_problem_predicates_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_function_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemFunctionDetails>(
    "problem_expert/get_problem_function", std::bind(
      &ProblemExpertNode::get_problem_function_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_functions_service_ = create_service<plansys2_msgs::srv::GetProblemFunctions>(
    "problem_expert/get_problem_functions",
    std::bind(
      &ProblemExpertNode::get_problem_functions_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_service_ = create_service<plansys2_msgs::srv::GetProblem>(
    "problem_expert/get_problem", std::bind(
      &ProblemExpertNode::get_problem_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  is_problem_goal_satisfied_service_ = create_service<plansys2_msgs::srv::IsProblemGoalSatisfied>(
    "problem_expert/is_problem_goal_satisfied", std::bind(
      &ProblemExpertNode::is_problem_goal_satisfied_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_goal_service_ = create_service<plansys2_msgs::srv::RemoveProblemGoal>(
    "problem_expert/remove_problem_goal",
    std::bind(
      &ProblemExpertNode::remove_problem_goal_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  clear_problem_knowledge_service_ = create_service<plansys2_msgs::srv::ClearProblemKnowledge>(
    "problem_expert/clear_problem_knowledge",
    std::bind(
      &ProblemExpertNode::clear_problem_knowledge_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_instance_service_ = create_service<plansys2_msgs::srv::RemoveProblemInstance>(
    "problem_expert/remove_problem_instance",
    std::bind(
      &ProblemExpertNode::remove_problem_instance_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_predicate_service_ = create_service<plansys2_msgs::srv::RemoveProblemPredicate>(
    "problem_expert/remove_problem_predicate",
    std::bind(
      &ProblemExpertNode::remove_problem_predicate_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_function_service_ = create_service<plansys2_msgs::srv::RemoveProblemFunction>(
    "problem_expert/remove_problem_function",
    std::bind(
      &ProblemExpertNode::remove_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  exist_problem_predicate_service_ = create_service<plansys2_msgs::srv::ExistProblemPredicate>(
    "problem_expert/exist_problem_predicate",
    std::bind(
      &ProblemExpertNode::exist_problem_predicate_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  exist_problem_function_service_ = create_service<plansys2_msgs::srv::ExistProblemFunction>(
    "problem_expert/exist_problem_function",
    std::bind(
      &ProblemExpertNode::exist_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  update_problem_function_service_ = create_service<plansys2_msgs::srv::UpdateProblemFunction>(
    "problem_expert/update_problem_function",
    std::bind(
      &ProblemExpertNode::update_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "problem_expert/update_notify",
    rclcpp::QoS(100));

  knowledge_pub_ = create_publisher<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge",
    rclcpp::QoS(100).transient_local());
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ProblemExpertNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  // (fmrico) Here we could have a discussion if we should read the domain from file or
  // from the domain_expert service. Then, we should configure first domain_expert node
  auto model_file = get_parameter("model_file").get_value<std::string>();

  auto model_files = tokenize(model_file, ":");

  std::ifstream domain_first_ifs(model_files[0]);
  std::string domain_first_str((
      std::istreambuf_iterator<char>(domain_first_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<DomainExpert>(domain_first_str);

  for (size_t i = 1; i < model_files.size(); i++) {
    std::ifstream domain_ifs(model_files[i]);
    std::string domain_str((
        std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());
    domain_expert->extendDomain(domain_str);
  }

  problem_expert_ = std::make_shared<ProblemExpert>(domain_expert);
  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  update_pub_->on_activate();
  knowledge_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  update_pub_->on_deactivate();
  knowledge_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
ProblemExpertNode::add_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    if (request->goal != "(and )") {
      parser::pddl::tree::Goal goal;
      goal.fromString(request->goal);
      response->success = problem_expert_->setGoal(goal);

      if (response->success) {
        update_pub_->publish(std_msgs::msg::Empty());
        knowledge_pub_->publish(*get_knowledge_as_msg());
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
  const std::shared_ptr<plansys2_msgs::srv::AddProblemInstance::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemInstance::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Instance instance;
    instance.name = request->instance;
    instance.type = request->type;

    response->success = problem_expert_->addInstance(instance);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info = "Instance not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemPredicate::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemPredicate::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Predicate predicate;
    predicate.name = request->predicate;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      predicate.parameters.push_back(param);
    }

    response->success = problem_expert_->addPredicate(predicate);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info =
        "Predicate [" + predicate.toString() + "] not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemFunction::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemFunction::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Function function;
    function.name = request->function;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      function.parameters.push_back(param);
    }

    function.value = request->value;

    response->success = problem_expert_->addFunction(function);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info = "Function not valid";
    }
  }
}

void
ProblemExpertNode::get_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->goal = problem_expert_->getGoal().toString();
  }
}

void
ProblemExpertNode::get_problem_instance_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto instance = problem_expert_->getInstance(request->instance);
    if (instance) {
      response->success = true;
      response->type = instance.value().type;
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
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto instances = problem_expert_->getInstances();
    response->success = true;
    for (auto const & instance : instances) {
      response->instances.push_back(instance.name);
      response->types.push_back(instance.type);
    }
  }
}

void
ProblemExpertNode::get_problem_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicateDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto params = problem_expert_->getPredicate(request->predicate);
    if (params) {
      response->name = params.value().name;

      for (const auto & param :  params.value().parameters) {
        response->param_names.push_back(param.name);
        response->param_types.push_back(param.type);
      }

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
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemPredicates::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto predicates = problem_expert_->getPredicates();
    response->success = true;
    for (const auto & predicate : predicates) {
      response->predicates.push_back(predicate.toString());
    }
  }
}

void
ProblemExpertNode::get_problem_function_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemFunctionDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemFunctionDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto params = problem_expert_->getFunction(request->function);
    if (params) {
      response->name = params.value().name;

      for (const auto & param : params.value().parameters) {
        response->param_names.push_back(param.name);
        response->param_types.push_back(param.type);
      }

      response->value = params.value().value;

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
  const std::shared_ptr<plansys2_msgs::srv::GetProblemFunctions::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemFunctions::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto functions = problem_expert_->getFunctions();
    response->success = true;
    for (const auto & function : functions) {
      response->functions.push_back(function.toString());
    }
  }
}

void
ProblemExpertNode::get_problem_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->problem = problem_expert_->getProblem();
  }
}

void
ProblemExpertNode::is_problem_goal_satisfied_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->satisfied = problem_expert_->isGoalSatisfied(parser::pddl::tree::Goal(request->goal));
  }
}

void
ProblemExpertNode::remove_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->clearGoal();

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info = "Error clearing goal";
    }
  }
}

void
ProblemExpertNode::clear_problem_knowledge_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->clearKnowledge();

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info = "Error clearing knowledge";
    }
  }
}


void
ProblemExpertNode::remove_problem_instance_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemInstance::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemInstance::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->removeInstance(request->instance);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info = "Error removing instance";
    }
  }
}

void
ProblemExpertNode::remove_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemPredicate::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemPredicate::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Predicate predicate;
    predicate.name = request->predicate;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      predicate.parameters.push_back(param);
    }

    response->success = problem_expert_->removePredicate(predicate);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());
    } else {
      response->error_info = "Error removing predicate";
    }
  }
}

void
ProblemExpertNode::remove_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemFunction::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemFunction::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Function function;
    function.name = request->function;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      function.parameters.push_back(param);
    }

    response->success = problem_expert_->removeFunction(function);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
    } else {
      response->error_info = "Error removing function";
    }
  }
}

void
ProblemExpertNode::exist_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ExistProblemPredicate::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ExistProblemPredicate::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->exist = false;
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Predicate predicate;
    predicate.name = request->predicate;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      predicate.parameters.push_back(param);
    }

    response->exist = problem_expert_->existPredicate(predicate);
  }
}

void
ProblemExpertNode::exist_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ExistProblemFunction::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ExistProblemFunction::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->exist = false;
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Function function;
    function.name = request->function;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      function.parameters.push_back(param);
    }

    response->exist = problem_expert_->existFunction(function);
  }
}

void
ProblemExpertNode::update_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::UpdateProblemFunction::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::UpdateProblemFunction::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    parser::pddl::tree::Function function;
    function.name = request->function;

    for (const auto & param_name : request->arguments) {
      parser::pddl::tree::Param param;
      param.name = param_name;
      function.parameters.push_back(param);
    }

    function.value = request->value;

    response->success = problem_expert_->updateFunction(function);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
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
    ret_msgs->predicates.push_back(predicate.toString());
  }

  ret_msgs->goal = problem_expert_->getGoal().toString();

  return ret_msgs;
}

}  // namespace plansys2
