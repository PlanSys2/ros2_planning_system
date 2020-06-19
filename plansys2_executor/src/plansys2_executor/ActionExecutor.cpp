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
#include <vector>

#include "plansys2_executor/ActionExecutor.hpp"

#include "plansys2_domain_expert/Types.hpp"

namespace plansys2
{

using ExecuteAction = plansys2_msgs::action::ExecuteAction;
using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;

ActionExecutor::ActionExecutor(const std::string & action)
: status_(STARTING), finished_(false)
{
  feedback_.progress = 0.0f;

  spin_node_ = std::make_shared<rclcpp::Node>(current_action_.name + "action_client");
  aux_node_ = std::make_shared<rclcpp::Node>(current_action_.name + "aux_action_client");
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(aux_node_);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(aux_node_);

  if (!update_current_action(action)) {
    status_ = EXECUTION_ERROR;
    finished_ = true;
    RCLCPP_ERROR(spin_node_->get_logger(), "Action client execution error initializing action");
    return;
  }

  if (check(current_action_.at_start_requirements)) {
    this->execute_action_client_ptr_ = rclcpp_action::create_client<ExecuteAction>(
      spin_node_->get_node_base_interface(),
      spin_node_->get_node_graph_interface(),
      spin_node_->get_node_logging_interface(),
      spin_node_->get_node_waitables_interface(),
      current_action_.name);

    if (!executeAction()) {
      status_ = EXECUTION_ERROR;
      finished_ = true;
    }
  } else {
    status_ = AT_START_REQ_ERROR;
    finished_ = true;
    RCLCPP_ERROR(spin_node_->get_logger(), "Action client execution error testing at_start reqs");
  }

  using namespace std::placeholders;
  update_problem_sub_ = spin_node_->create_subscription<std_msgs::msg::Empty>(
    "/problem_expert/update_notify",
    10,
    std::bind(&ActionExecutor::update_callback, this, _1));
}

void
ActionExecutor::update_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (!check(current_action_.over_all_requirements)) {
    status_ = OVER_ALL_REQ_ERROR;
    finished_ = true;
    RCLCPP_ERROR(spin_node_->get_logger(), "Action client execution error testing over_all reqs");
    auto future_cancel_all = this->execute_action_client_ptr_->async_cancel_all_goals();
  }
}

ActionExecutor::ActionExecutor()
{
  std::cerr << "This constructor must be used only for testing" << std::endl;
}

bool
ActionExecutor::executeAction()
{
  if (!apply(current_action_.at_start_effects)) {
    status_ = AT_START_EF_ERROR;
    finished_ = true;
    RCLCPP_ERROR(spin_node_->get_logger(),
      "Action client execution error applying at_start effects");
    return false;
  }

  if (!check(current_action_.over_all_requirements)) {
    status_ = OVER_ALL_REQ_ERROR;
    finished_ = true;
    RCLCPP_ERROR(spin_node_->get_logger(),
      "Action client execution error testing over_all reqs");
    return false;
  }

  using namespace std::placeholders;

  feedback_.progress = 0;

  result_.success = false;
  result_.error_info = "";

  finished_ = false;

  if (!this->execute_action_client_ptr_) {
    RCLCPP_ERROR(spin_node_->get_logger(), "Action client not initialized");
  }

  if (!this->execute_action_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(spin_node_->get_logger(), "Action server not available after waiting");
    return false;
  }

  auto goal_msg = ExecuteAction::Goal();
  goal_msg.action = current_action_.name;
  for (const auto & param : current_action_.parameters) {
    goal_msg.arguments.push_back(param.name);
  }

  auto send_goal_options = rclcpp_action::Client<ExecuteAction>::SendGoalOptions();

  send_goal_options.feedback_callback =
    std::bind(&ActionExecutor::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&ActionExecutor::result_callback, this, _1);
  auto goal_handle_future = this->execute_action_client_ptr_->async_send_goal(
    goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(spin_node_, goal_handle_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(spin_node_->get_logger(), "send_goal failed");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      spin_node_->get_logger(),
      "ActionExecutor: Plan execution was rejected by the action server");
    return false;
  }

  return true;
}

void
ActionExecutor::update()
{
  if (rclcpp::ok() && !finished_) {
    rclcpp::spin_some(spin_node_->get_node_base_interface());
  }
}

void
ActionExecutor::feedback_callback(
  GoalHandleExecuteAction::SharedPtr,
  const std::shared_ptr<const ExecuteAction::Feedback> feedback)
{
  feedback_ = *feedback;
}

void
ActionExecutor::result_callback(const GoalHandleExecuteAction::WrappedResult & result)
{
  finished_ = true;
  result_ = *result.result;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(spin_node_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(spin_node_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(spin_node_->get_logger(), "Unknown result code");
      return;
  }

  if (result.result->success) {
    RCLCPP_DEBUG(spin_node_->get_logger(), "Result action received: Success");

    if (!check(current_action_.at_end_requirements)) {
      status_ = AT_END_REQ_ERROR;
      finished_ = true;
      RCLCPP_ERROR(spin_node_->get_logger(), "Action client execution error testing at_end reqs");
    } else if (apply(current_action_.at_end_effects)) {
      status_ = SUCCEDED;
      finished_ = true;
    } else {
      status_ = AT_END_EF_ERROR;
      finished_ = true;
      RCLCPP_ERROR(spin_node_->get_logger(),
        "Action client execution error applying at_end effects");
    }
  } else {
    finished_ = true;
    status_ = EXECUTION_ERROR;
    RCLCPP_WARN(spin_node_->get_logger(), "Result action received: Fail [%s]",
      result.result->error_info.c_str());
  }
}

std::string
ActionExecutor::get_name(const std::string & action_expr)
{
  std::string working_action_expr = getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  return working_action_expr.substr(0, delim);
}

std::vector<std::string>
ActionExecutor::get_params(const std::string & action_expr)
{
  std::vector<std::string> ret;

  std::string working_action_expr = getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  working_action_expr = working_action_expr.substr(delim + 1);

  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = working_action_expr.find(" ", start);
    auto param = working_action_expr.substr(
      start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

bool
ActionExecutor::update_current_action(const std::string & action_expr)
{
  current_action_.name = get_name(action_expr);

  current_action_.parameters.clear();
  for (const auto & param : get_params(action_expr)) {
    current_action_.parameters.push_back(Param{param, ""});
  }

  auto action = domain_client_->getAction(current_action_.name);
  auto durative_action = domain_client_->getDurativeAction(current_action_.name);

  if (action) {
    auto at_start_req = action.value().preconditions.toString();
    auto at_end_eff = action.value().effects.toString();


    for (size_t i = 0; i < current_action_.parameters.size(); i++) {
      std::string pattern = "?" + std::to_string(i);
      int pos;
      while ((pos = at_start_req.find(pattern)) != std::string::npos) {
        at_start_req.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
      while ((pos = at_end_eff.find(pattern)) != std::string::npos) {
        at_end_eff.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
    }

    current_action_.at_start_requirements.fromString(at_start_req);
    current_action_.over_all_requirements.fromString("");
    current_action_.at_end_requirements.fromString("");
    current_action_.at_start_effects.fromString("");
    current_action_.at_end_effects.fromString(at_end_eff);

    return true;
  }

  if (durative_action) {
    auto at_start_req = durative_action.value().at_start_requirements.toString();
    auto over_all_req = durative_action.value().over_all_requirements.toString();
    auto at_end_req = durative_action.value().at_end_requirements.toString();
    auto at_start_eff = durative_action.value().at_start_effects.toString();
    auto at_end_eff = durative_action.value().at_end_effects.toString();


    for (size_t i = 0; i < current_action_.parameters.size(); i++) {
      std::string pattern = "?" + std::to_string(i);
      int pos;
      while ((pos = at_start_req.find(pattern)) != std::string::npos) {
        at_start_req.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
      while ((pos = over_all_req.find(pattern)) != std::string::npos) {
        over_all_req.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
      while ((pos = at_end_req.find(pattern)) != std::string::npos) {
        at_end_req.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
      while ((pos = at_start_eff.find(pattern)) != std::string::npos) {
        at_start_eff.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
      while ((pos = at_end_eff.find(pattern)) != std::string::npos) {
        at_end_eff.replace(pos, pattern.length(), current_action_.parameters[i].name);
      }
    }

    current_action_.at_start_requirements.fromString(at_start_req);
    current_action_.over_all_requirements.fromString(over_all_req);
    current_action_.at_end_requirements.fromString(at_end_req);
    current_action_.at_start_effects.fromString(at_start_eff);
    current_action_.at_end_effects.fromString(at_end_eff);

    return true;
  }

  RCLCPP_ERROR(spin_node_->get_logger(), "Action [%s] not found",
    current_action_.name.c_str());
  return false;
}

bool
ActionExecutor::check(const std::shared_ptr<TreeNode> node) const
{
  if (node == nullptr) {  // No req expression
    return true;
  }

  switch (node->type_) {
    case AND: {
        std::shared_ptr<plansys2::AndNode> pn_and =
          std::dynamic_pointer_cast<plansys2::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          ret = ret && check(op);
        }
        return ret;
      }

    case OR: {
        std::shared_ptr<plansys2::OrNode> pn_or =
          std::dynamic_pointer_cast<plansys2::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          ret = ret || check(op);
        }
        return ret;
      }

    case NOT: {
        std::shared_ptr<plansys2::NotNode> pn_not =
          std::dynamic_pointer_cast<NotNode>(node);

        return !check(pn_not->op);
      }

    case PREDICATE: {
        std::shared_ptr<plansys2::PredicateNode> pred =
          std::dynamic_pointer_cast<PredicateNode>(node);

        return problem_client_->existPredicate(pred->predicate_);
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }
}


bool
ActionExecutor::apply(const std::shared_ptr<TreeNode> node, bool negate) const
{
  if (node == nullptr) {  // No apply expression
    return true;
  }

  switch (node->type_) {
    case AND: {
        std::shared_ptr<plansys2::AndNode> pn_and =
          std::dynamic_pointer_cast<plansys2::AndNode>(node);
        bool ret = true;

        for (const auto & op : pn_and->ops) {
          ret = ret && apply(op, negate);
        }
        return ret;
      }

    case OR: {
        std::shared_ptr<plansys2::OrNode> pn_or =
          std::dynamic_pointer_cast<plansys2::OrNode>(node);
        bool ret = true;

        for (const auto & op : pn_or->ops) {
          ret = ret && apply(op, negate);
        }
        return ret;
      }

    case NOT: {
        std::shared_ptr<plansys2::NotNode> pn_not =
          std::dynamic_pointer_cast<NotNode>(node);

        return apply(pn_not->op, !negate);
      }

    case PREDICATE: {
        std::shared_ptr<plansys2::PredicateNode> pred =
          std::dynamic_pointer_cast<PredicateNode>(node);

        if (negate) {
          auto success = problem_client_->removePredicate(pred->predicate_);
          return success;
        } else {
          auto success = problem_client_->addPredicate(pred->predicate_);
          return success;
        }
      }

    default:
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion [" <<
        node->toString() << "]" << std::endl;
  }
}

}  // namespace plansys2
