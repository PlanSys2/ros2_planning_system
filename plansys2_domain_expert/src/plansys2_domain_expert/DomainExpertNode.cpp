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

#include <future>
#include <string>
#include <memory>
#include <vector>

#include "plansys2_core/ServiceCall.hpp"
#include "kairos/kairos.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_msgs/msg/node.hpp"

namespace plansys2
{

DomainExpertNode::DomainExpertNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("domain_expert", options)
{
  declare_parameter("model_file", "");
  declare_parameter("validate_using_planner_node", false);

  // Its own group, served by the executor of the node while a callback waits for the planner
  validate_domain_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  problem_update_domain_client_ = create_client<plansys2_msgs::srv::ChangeDomain>(
    "problem_expert/update_domain");

  get_name_service_ = create_service<plansys2_msgs::srv::GetDomainName>(
    "domain_expert/get_domain_name",
    guarded<plansys2_msgs::srv::GetDomainName>(
      get_logger(),
      std::bind(
        &DomainExpertNode::get_domain_name_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));
  get_types_service_ = create_service<plansys2_msgs::srv::GetDomainTypes>(
    "domain_expert/get_domain_types",
    guarded<plansys2_msgs::srv::GetDomainTypes>(
      get_logger(),
      std::bind(
        &DomainExpertNode::get_domain_types_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));
  get_constants_service_ = create_service<plansys2_msgs::srv::GetDomainConstants>(
    "domain_expert/get_domain_constants",
    guarded<plansys2_msgs::srv::GetDomainConstants>(
      get_logger(),
      std::bind(
        &DomainExpertNode::get_domain_constants_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));
  // PDDL+: processes and events, served as actions are
  using Names = plansys2_msgs::srv::GetDomainActions;
  using Details = plansys2_msgs::srv::GetDomainActionDetails;
  auto list = [this](bool processes) {
      return [this, processes](
        const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<Names::Request>,
        const std::shared_ptr<Names::Response> response) {
               if (domain_expert_ == nullptr) {
                 response->success = false;
                 response->error_info = plansys2::kNotActive;
                 return;
               }
               response->success = true;
               response->actions = processes ? domain_expert_->getProcesses() :
                 domain_expert_->getEvents();
             };
    };
  auto details = [this](bool processes) {
      return [this, processes](
        const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<Details::Request> request,
        const std::shared_ptr<Details::Response> response) {
               if (domain_expert_ == nullptr) {
                 response->success = false;
                 response->error_info = plansys2::kNotActive;
                 return;
               }
               plansys2_msgs::msg::Action::SharedPtr found;
               try {
                 found = processes ?
                   domain_expert_->getProcess(request->action, request->parameters) :
                   domain_expert_->getEvent(request->action, request->parameters);
               } catch (const std::exception & e) {
                 response->success = false;
                 response->error_info = e.what();
                 return;
               }
               response->success = found != nullptr;
               if (found) {
                 response->action = *found;
               } else {
                 response->error_info = processes ? "Process not found" : "Event not found";
               }
             };
    };
  get_domain_processes_service_ =
    create_service<Names>("domain_expert/get_domain_processes",
    guarded<Names>(get_logger(), list(true)));
  get_domain_process_details_service_ =
    create_service<Details>("domain_expert/get_domain_process_details",
    guarded<Details>(get_logger(), details(true)));
  get_domain_events_service_ =
    create_service<Names>("domain_expert/get_domain_events",
    guarded<Names>(get_logger(), list(false)));
  get_domain_event_details_service_ =
    create_service<Details>("domain_expert/get_domain_event_details",
    guarded<Details>(get_logger(), details(false)));

  get_domain_actions_service_ = create_service<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_actions",
    guarded<plansys2_msgs::srv::GetDomainActions>(
      get_logger(),
      std::bind(
        &DomainExpertNode::get_domain_actions_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));
  get_domain_action_details_service_ =
    create_service<plansys2_msgs::srv::GetDomainActionDetails>(
    "domain_expert/get_domain_action_details",
    guarded<plansys2_msgs::srv::GetDomainActionDetails>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_action_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_durative_actions_service_ = create_service<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_durative_actions",
    guarded<plansys2_msgs::srv::GetDomainActions>(
      get_logger(),
      std::bind(
        &DomainExpertNode::get_domain_durative_actions_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));
  get_domain_durative_action_details_service_ =
    create_service<plansys2_msgs::srv::GetDomainDurativeActionDetails>(
    "domain_expert/get_domain_durative_action_details",
    guarded<plansys2_msgs::srv::GetDomainDurativeActionDetails>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_durative_action_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_predicates_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "domain_expert/get_domain_predicates",
    guarded<plansys2_msgs::srv::GetStates>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_predicates_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "domain_expert/get_domain_predicate_details",
    guarded<plansys2_msgs::srv::GetNodeDetails>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_functions_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "domain_expert/get_domain_functions",
    guarded<plansys2_msgs::srv::GetStates>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_functions_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_function_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "domain_expert/get_domain_function_details",
    guarded<plansys2_msgs::srv::GetNodeDetails>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_function_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_derived_predicates_service_ =
    create_service<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>(
    "domain_expert/get_domain_derived_predicates",
    guarded<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_derived_predicates_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_derived_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>(
    "domain_expert/get_domain_derived_predicate_details",
    guarded<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_derived_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  get_domain_service_ = create_service<plansys2_msgs::srv::GetDomain>(
    "domain_expert/get_domain",
    guarded<plansys2_msgs::srv::GetDomain>(get_logger(), std::bind(
      &DomainExpertNode::get_domain_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3)));
  change_domain_service_ = create_service<plansys2_msgs::srv::ChangeDomain>(
    "domain_expert/change_domain",
    guarded_deferred<plansys2_msgs::srv::ChangeDomain>(
      get_logger(),
      [this](
        std::shared_ptr<rclcpp::Service<plansys2_msgs::srv::ChangeDomain>> service,
        std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<plansys2_msgs::srv::ChangeDomain::Request> request) {
        change_domain_service_callback(service, header, request);
    }));
  domain_pub_ = create_publisher<std_msgs::msg::String>(
    "domain_expert/domain",
    rclcpp::QoS(100).transient_local());
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
DomainExpertNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  const std::string model_file = get_parameter("model_file").get_value<std::string>();
  const bool validate_using_planner_node =
    get_parameter("validate_using_planner_node").get_value<bool>();

  auto model_files = tokenize(model_file, ":");

  if (validate_using_planner_node) {
    validate_domain_client_ = create_client<plansys2_msgs::srv::ValidateDomain>(
      "planner/validate_domain", plansys2::services_qos(), validate_domain_callback_group_);
    if (!wait_for_server(validate_domain_client_, std::chrono::seconds(5))) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        validate_domain_client_->get_service_name() << " not available: is the planner running?");
      validate_domain_client_.reset();
      return CallbackReturnT::FAILURE;
    }
  } else {
    validate_locally_ = true;
  }

  // Build the joint domain from all model_files first (extendDomain() merges
  // predicates/actions/etc, not naive string concatenation), then validate the whole
  // joint result once via validateDomain() — the same helper
  // change_domain_service_callback() uses — instead of duplicating the
  // validate-via-planner-or-locally logic here too.
  std::shared_ptr<DomainExpert> joint_domain_expert;
  std::string single_domain;  // one model file is validated as written
  for (size_t i = 0; i < model_files.size(); i++) {
    std::ifstream domain_ifs(model_files[i]);
    std::string domain_str((
        std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());

    if (i == 0) {
      joint_domain_expert = std::make_shared<DomainExpert>(domain_str);
      single_domain = domain_str;
    } else {
      joint_domain_expert->extendDomain(domain_str);
    }
  }

  std::string error;
  const std::string to_validate =
    model_files.size() == 1 ? single_domain : joint_domain_expert->getDomain();
  if (!validateDomain(to_validate, domain_expert_, error)) {
    RCLCPP_ERROR_STREAM(get_logger(), error);
    return CallbackReturnT::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  domain_pub_->on_activate();

  std_msgs::msg::String domain_msg;
  domain_msg.data = domain_expert_->getDomain();
  domain_pub_->publish(domain_msg);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  domain_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  domain_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
DomainExpertNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
DomainExpertNode::get_domain_name_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainName::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainName::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->name = domain_expert_->getName();
  }
}

void
DomainExpertNode::get_domain_types_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainTypes::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->types = domain_expert_->getTypes();
  }
}

void
DomainExpertNode::get_domain_constants_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainConstants::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainConstants::Response> response)
{
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->constants = domain_expert_->getConstants(request->type);
  }
}

void
DomainExpertNode::get_domain_actions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    for (const auto & action : domain_expert_->getActions()) {
      response->actions.push_back(action);
    }
  }
}

void
DomainExpertNode::get_domain_action_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActionDetails::Response> response)
{
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;

    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    // A construct PlanSys2 cannot represent must not take the node down
    plansys2_msgs::msg::Action::SharedPtr action;
    try {
      action = domain_expert_->getAction(request->action, request->parameters);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      response->success = false;
      response->error_info = e.what();
      return;
    }

    if (action) {
      response->action = *action;
      response->success = true;
    } else {
      RCLCPP_WARN(get_logger(), "Requesting a non-existing action [%s]", request->action.c_str());
      response->success = false;
      response->error_info = "Action not found";
    }
  }
}

void
DomainExpertNode::get_domain_durative_actions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainActions::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    for (const auto & action : domain_expert_->getDurativeActions()) {
      response->actions.push_back(action);
    }
  }
}

void
DomainExpertNode::get_domain_durative_action_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDurativeActionDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDurativeActionDetails::Response> response)
{
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;

    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    // A construct PlanSys2 cannot represent must not take the node down
    plansys2_msgs::msg::DurativeAction::SharedPtr action;
    try {
      action = domain_expert_->getDurativeAction(request->durative_action, request->parameters);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      response->success = false;
      response->error_info = e.what();
      return;
    }

    if (action) {
      response->durative_action = *action;
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Requesting a non-existing durative action [%s]",
        request->durative_action.c_str());
      response->success = false;
      response->error_info = "Durative action not found";
    }
  }
}

void
DomainExpertNode::get_domain_predicates_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
      domain_expert_->getPredicates());
  }
}

void
DomainExpertNode::get_domain_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    auto predicate = domain_expert_->getPredicate(request->expression);
    if (predicate) {
      response->node = predicate.value();
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Requesting a non-existing predicate [%s]",
        request->expression.c_str());
      response->success = false;
      response->error_info = "Predicate not found";
    }
  }
}

void
DomainExpertNode::get_domain_functions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Function>(
      domain_expert_->getFunctions());
  }
}

void
DomainExpertNode::get_domain_function_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    auto function = domain_expert_->getFunction(request->expression);
    if (function) {
      response->node = function.value();
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Requesting a non-existing function [%s]",
        request->expression.c_str());
      response->success = false;
      response->error_info = "Function not found";
    }
  }
}

void
DomainExpertNode::get_domain_derived_predicates_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;
    response->predicates = domain_expert_->getDerivedPredicates();
  }
}

void
DomainExpertNode::get_domain_derived_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Response> response)
{
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;

    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    // A construct PlanSys2 cannot represent must not take the node down
    std::vector<plansys2_msgs::msg::Derived> predicates;
    try {
      predicates = domain_expert_->getDerivedPredicate(request->predicate);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      response->success = false;
      response->error_info = e.what();
      return;
    }

    if (predicates.size() > 0) {
      response->predicates = predicates;
      response->success = true;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Requesting a non-existing derived predicate [%s]",
        request->predicate.c_str());
      response->success = false;
      response->error_info = "Derived predicate not found";
    }
  }
}

void
DomainExpertNode::get_domain_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetDomain::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetDomain::Response> response)
{
  (void)request;
  (void)request_header;
  if (domain_expert_ == nullptr) {
    response->success = false;
    response->error_info = plansys2::kNotActive;
    RCLCPP_WARN(get_logger(), "%s", plansys2::kNotActive);
  } else {
    response->success = true;

    std::ostringstream stream;
    stream << domain_expert_->getDomain();
    response->domain = stream.str();
  }
}

bool
DomainExpertNode::validateDomain(
  const std::string & domain,
  std::shared_ptr<DomainExpert> & out_domain_expert,
  std::string & error)
{
  auto candidate = std::make_shared<DomainExpert>(domain);

  // The planner may take what PlanSys2 cannot keep or execute: the same check at startup as in
  // a change, and at load rather than when a plan uses it
  if (!candidate->getParseError().empty()) {
    error = "PlanSys2 cannot read this domain: " + candidate->getParseError();
    return false;
  }
  const auto unsupported = candidate->getUnsupported();
  if (!unsupported.empty()) {
    error = "PlanSys2 cannot execute this domain:";
    for (const auto & line : unsupported) {
      error += "\n  " + line;
    }
    return false;
  }

  bool check_valid = true;
  if (validate_domain_client_) {
    auto request = std::make_shared<plansys2_msgs::srv::ValidateDomain::Request>();
    request->domain = candidate->getDomain();
    auto future_result = validate_domain_client_->async_send_request(std::move(request));
    // The executor of the node brings the answer (multi-threaded: we may be in a callback)
    if (future_result.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      validate_domain_client_->remove_pending_request(future_result);
      error = "Timed out waiting for service: " +
        std::string(validate_domain_client_->get_service_name());
      return false;
    }
    auto response = future_result.get();
    check_valid = response->success;
    if (!check_valid) {
      error = response->error_info.empty() ? "PDDL syntax error" : response->error_info;
    }
  } else if (validate_locally_) {
    // Kairos checks the PDDL as given (syntax, declarations, types, arities), with no
    // planner process, and says where each problem is. Not the text DomainExpert rebuilds:
    // its parser is lenient, e.g. it closes a missing parenthesis
    const std::string & text = domain;
    const auto report = kairos::check_domain(text);
    for (const auto & d : report.diagnostics) {
      const auto message = kairos::to_string(d, text);
      if (d.severity == kairos::Severity::Error) {
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        if (error.empty()) {error = message;}
      } else if (d.severity == kairos::Severity::Warning) {
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
      }
    }
    check_valid = report.valid;
    if (!check_valid && error.empty()) {
      error = "PDDL syntax error";
    }
  } else {
    error = "domain_expert has no domain validator configured";
    return false;
  }

  if (!check_valid) {
    return false;
  }

  out_domain_expert = candidate;
  return true;
}

void
DomainExpertNode::take_domain(std::shared_ptr<DomainExpert> domain_expert)
{
  // Swap in the instance validateDomain() already parsed instead of parsing again
  domain_expert_ = domain_expert;

  std_msgs::msg::String domain_msg;
  domain_msg.data = domain_expert_->getDomain();
  domain_pub_->publish(domain_msg);

  RCLCPP_INFO(get_logger(), "[%s] Domain changed", get_name());
}

void
DomainExpertNode::change_domain_service_callback(
  std::shared_ptr<rclcpp::Service<plansys2_msgs::srv::ChangeDomain>> service,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<plansys2_msgs::srv::ChangeDomain::Request> request)
{
  auto respond = [this, service, request_header](bool success, const std::string & error) {
      plansys2_msgs::srv::ChangeDomain::Response response;
      response.success = success;
      response.error_info = error;
      if (!success) {
        RCLCPP_WARN_STREAM(
          get_logger(), "[" << get_name() << "] Rejected domain change: " << error);
      }
      service->send_response(*request_header, response);
    };

  // Subscribers only learn about the change from domain_pub_, so it must be active
  if (domain_expert_ == nullptr || !domain_pub_->is_activated()) {
    respond(false, plansys2::kNotActive);
    return;
  }

  if (request->domain.find_first_not_of(" \t\r\n") == std::string::npos) {
    respond(false, "Empty domain");
    return;
  }

  if (change_in_progress_) {
    respond(false, "Another domain change is in progress");
    return;
  }

  std::shared_ptr<DomainExpert> validated_domain_expert;
  std::string error;
  if (!validateDomain(request->domain, validated_domain_expert, error)) {
    respond(false, error);
    return;
  }

  // Without a problem expert there is nobody to wait for
  if (!problem_update_domain_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "No problem expert: the domain changes without it");
    take_domain(validated_domain_expert);
    respond(true, "");
    return;
  }

  // The problem expert first, so that when we answer it already works with the new domain;
  // if it cannot take it, nothing changes
  change_in_progress_ = true;
  auto answered = std::make_shared<bool>(false);
  auto update = std::make_shared<plansys2_msgs::srv::ChangeDomain::Request>();
  update->domain = validated_domain_expert->getDomain();
  auto pending = problem_update_domain_client_->async_send_request(
    update,
    [this, answered, respond, validated_domain_expert](
      rclcpp::Client<plansys2_msgs::srv::ChangeDomain>::SharedFuture future) {
      if (*answered) {
        return;
      }
      *answered = true;
      change_in_progress_ = false;
      change_timeout_.reset();
      auto result = future.get();
      if (!result->success) {
        respond(
          false, result->error_info.empty() ? "The problem expert could not take the domain" :
          result->error_info);
        return;
      }
      take_domain(validated_domain_expert);
      respond(true, "");
    });

  auto request_id = pending.request_id;
  change_timeout_ = create_wall_timer(
    std::chrono::seconds(5), [this, answered, respond, request_id]() {
      change_timeout_->cancel();
      if (*answered) {
        return;
      }
      *answered = true;
      change_in_progress_ = false;
      problem_update_domain_client_->remove_pending_request(request_id);
      respond(false, "The problem expert did not answer");
    });
}

}  // namespace plansys2
