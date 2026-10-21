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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

namespace plansys2
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using ExecutePlan = plansys2_msgs::action::ExecutePlan;

ExecutorClient::ExecutorClient(ClientNode::Interfaces node)
: node_(std::make_shared<ClientNode>(node)),
  active_(std::make_shared<ActiveComponent>(node_, "executor"))
{
  action_client_ = make_action_client(node);

  namespace srv = plansys2_msgs::srv;
  get_ordered_sub_goals_client_ = std::make_shared<ServiceClient<srv::GetOrderedSubGoals>>(
    node_, "executor/get_ordered_sub_goals", active_);
  get_plan_client_ =
    std::make_shared<ServiceClient<srv::GetPlan>>(node_, "executor/get_plan", active_);
  get_remaining_plan_client_ =
    std::make_shared<ServiceClient<srv::GetPlan>>(node_, "executor/get_remaining_plan", active_);
  get_replanning_problem_client_ = std::make_shared<ServiceClient<srv::GetProblem>>(
    node_, "executor/get_replanning_problem", active_);
}

bool
ExecutorClient::start_plan_execution(const plansys2_msgs::msg::Plan & plan)
{
  auto success = on_new_goal_received(plan);

  if (success) {
    executing_plan_ = true;
    sent_plan_ = plan;
    not_running_since_.reset();
    plan_check_.reset();
    result_check_.reset();
    result_attempts_ = 0;
    recovering_ = false;
    return true;
  }

  return false;
}

bool
ExecutorClient::execute_and_check_plan()
{
  // Feedback and result come while the client serves its group: a copy of what came so far
  node_->serve();
  bool available = false;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    available = goal_result_available_;
    goal_result_available_ = false;
    result = result_;
  }

  if (!available) {
    // A gone executor never answers: do not wait for it forever (#446)
    if (executing_plan_ && !action_client_->action_server_is_ready()) {
      const auto now = std::chrono::steady_clock::now();
      if (!server_lost_since_) {
        server_lost_since_ = now;
      } else if (now - server_lost_since_.value() > std::chrono::seconds(2)) {
        RCLCPP_ERROR(
          node_->get_logger(), "execute_and_check_plan: the executor is gone, the plan is lost");
        executing_plan_ = false;
        server_lost_since_.reset();
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult();
        return false;
      }
    } else {
      server_lost_since_.reset();
      // Not running our plan: ended with its result lost on the way, or a restarted executor
      // that has a server, but not our goal, whose result would never come
      if (executing_plan_ && (recovering_ || plan_not_running())) {
        recovering_ = true;
        const auto recovery = recover_result(result);
        if (recovery == Recovery::Pending) {
          return true;
        }
        recovering_ = false;
        if (recovery == Recovery::Lost) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "execute_and_check_plan: the executor does not run the plan (restarted?), it is lost");
          executing_plan_ = false;
          std::lock_guard<std::mutex> lock(mutex_);
          result_ = rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult();
          return false;
        }
        RCLCPP_WARN(
          node_->get_logger(), "execute_and_check_plan: the result was lost, asked for again");
        available = true;
      }
    }
    if (!available) {
      return true;  // Plan not finished
    }
  } else {
    recovering_ = false;
    result_check_.reset();
  }

  switch (result.code) {
    // Failed and preempted plans end as aborted goals (#436)
    case rclcpp_action::ResultCode::SUCCEEDED:
    case rclcpp_action::ResultCode::ABORTED:
      if (result.result == nullptr) {
        RCLCPP_WARN(
          node_->get_logger(),
          "execute_and_check_plan: Plan failed due to a nullptr in the result");
      } else if (result.result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "execute_and_check_plan: Plan Succeeded");
      } else if (result.result->result == plansys2_msgs::action::ExecutePlan::Result::PREEMPT) {
        RCLCPP_INFO(node_->get_logger(), "execute_and_check_plan: Plan Preempted");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "execute_and_check_plan: Plan Failed");
        for (auto msg : result.result->action_execution_status) {
          switch (msg.status) {
            case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " succeeded with message_status: " <<
                  msg.message_status);
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
              RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " failed with message_status: " <<
                  msg.message_status);
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was not executed");
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was cancelled");
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was executing");
          }
        }
      }
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Plan Cancelled");
      break;

    default:
      throw std::logic_error("ExecutorClient::executePlan: invalid status value");
  }

  // A preempted plan goes on with the new one
  if (!result.result ||
    result.result->result != plansys2_msgs::action::ExecutePlan::Result::PREEMPT)
  {
    executing_plan_ = false;
  }

  return false;  // Plan finished
}

bool
ExecutorClient::on_new_goal_received(const plansys2_msgs::msg::Plan & plan)
{
  auto goal = ExecutePlan::Goal();
  goal.plan = plan;

  // The result of a previous plan does not belong to this one
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult();
    goal_result_available_ = false;
    goal_handler_.reset();
  }
  server_lost_since_.reset();

  if (!active_->wait()) {
    return false;
  }

  auto send_goal_options = rclcpp_action::Client<ExecutePlan>::SendGoalOptions();

  send_goal_options.feedback_callback =
    std::bind(&ExecutorClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&ExecutorClient::result_callback, this, _1);

  // Set while the client serves its group, before any result of the goal can come
  send_goal_options.goal_response_callback =
    [this](const GoalHandleExecutePlan::SharedPtr & goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handler_ = goal_handle;
    };

  // As every call: until the executor is active, and once again if its action server goes
  auto server = [&]() {
      return wait_for_server([&]() {return action_client_->wait_for_action_server(3s);});
    };
  for (int attempt = 1; attempt <= kGoalAttempts && rclcpp::ok(); attempt++) {
    if (!active_->wait()) {
      return false;
    }
    if (!server()) {
      active_->lost();
      if (!active_->wait() || !server()) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        return false;
      }
    }
    auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);

    if (node_->wait(future_goal_handle, 3s) != rclcpp::FutureReturnCode::SUCCESS) {
      // The answer lost (the executor may run the plan already): again with a new client. A new
      // goal replaces the running one, which goes on from where it is, keeping what runs
      RCLCPP_WARN(
        node_->get_logger(), "execute_plan: no answer to the goal, sending it again (%d/%d)",
        attempt + 1, kGoalAttempts);
      auto node = node_->lock();
      if (!node) {
        return false;
      }
      action_client_ = make_action_client(*node);
      continue;
    }

    if (!future_goal_handle.get()) {
      // The executor rejects goals only while it is not active: wait for it, and again
      RCLCPP_WARN(node_->get_logger(), "execute_plan: goal rejected, the executor is not active");
      active_->lost();
      continue;
    }
    return true;
  }
  RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
  active_->lost();  // The next call checks it is still active
  return false;
}

bool
ExecutorClient::should_cancel_goal()
{
  if (!executing_plan_) {
    return false;
  }

  GoalHandleExecutePlan::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle = goal_handler_;
  }
  if (!goal_handle) {
    return false;
  }
  // Without our lock: the goal handle takes its own, and calls our callbacks holding it
  auto status = goal_handle->get_status();

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

void
ExecutorClient::cancel_plan_execution()
{
  if (should_cancel_goal()) {
    GoalHandleExecutePlan::SharedPtr goal_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle = goal_handler_;
    }
    auto future_cancel = action_client_->async_cancel_goal(goal_handle);
    if (node_->wait(future_cancel, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to cancel action server for execute_plan");
    }
  }

  executing_plan_ = false;
  std::lock_guard<std::mutex> lock(mutex_);
  goal_result_available_ = false;
}

std::vector<plansys2_msgs::msg::Tree> ExecutorClient::getOrderedSubGoals()
{
  std::vector<plansys2_msgs::msg::Tree> ret;

  auto request = std::make_shared<plansys2_msgs::srv::GetOrderedSubGoals::Request>();

  auto response = get_ordered_sub_goals_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return ret;
  }

  auto result = *response;

  if (result.success) {
    ret = result.sub_goals;
  } else {
    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      get_ordered_sub_goals_client_->name() << ": " <<
        result.error_info);
  }

  return ret;
}

std::optional<plansys2_msgs::msg::Plan> ExecutorClient::get_plan()
{
  auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();

  auto response = get_plan_client_->call(request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return result.plan;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_plan_client_->name() << ": " <<
        result.error_info);
    return {};
  }
}

std::optional<std::string> ExecutorClient::get_replanning_problem()
{
  auto response = get_replanning_problem_client_->call(
    std::make_shared<plansys2_msgs::srv::GetProblem::Request>(), std::chrono::seconds(5));
  if (!response || !response->success) {
    if (response) {
      RCLCPP_ERROR(node_->get_logger(), "get_replanning_problem: %s", response->error_info.c_str());
    }
    return {};
  }
  return response->problem;
}

std::optional<plansys2_msgs::msg::Plan> ExecutorClient::get_remaining_plan()
{
  auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();

  auto response = get_remaining_plan_client_->call(
    request, std::chrono::seconds(1));
  if (!response) {
    return {};
  }

  auto result = *response;

  if (result.success) {
    return result.plan;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_remaining_plan_client_->name() << ": " <<
        result.error_info);
    return {};
  }
}

void
ExecutorClient::feedback_callback(
  GoalHandleExecutePlan::SharedPtr goal_handle,
  const std::shared_ptr<const ExecutePlan::Feedback> feedback)
{
  (void)goal_handle;
  std::lock_guard<std::mutex> lock(mutex_);
  feedback_ = *feedback;
}

void
ExecutorClient::result_callback(const GoalHandleExecutePlan::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handler_ && goal_handler_->get_goal_id() == result.goal_id) {
    goal_result_available_ = true;
    result_ = result;
    feedback_ = ExecutePlan::Feedback();
  }
}

std::optional<ExecutePlan::Result>
ExecutorClient::getResult()
{
  node_->serve();
  std::lock_guard<std::mutex> lock(mutex_);
  if (result_.result != nullptr) {
    return *result_.result;
  } else {
    return {};
  }
}

rclcpp_action::Client<ExecutorClient::ExecutePlan>::SharedPtr
ExecutorClient::make_action_client(ClientNode::Interfaces node)
{
  return rclcpp_action::create_client<ExecutePlan>(
    node.get_node_base_interface(), node.get_node_graph_interface(),
    node.get_node_logging_interface(), node.get_node_waitables_interface(), "execute_plan",
    node_->group());
}

bool
ExecutorClient::plan_not_running()
{
  // Asked once a second, without waiting: the answer is read in a later call
  const auto now = std::chrono::steady_clock::now();
  if (!plan_check_client_) {
    plan_check_client_ = node_->create_client<plansys2_msgs::srv::GetPlan>("executor/get_plan");
  }
  if (plan_check_ && plan_check_->future.wait_for(0s) == std::future_status::ready) {
    auto response = plan_check_->future.get();
    plan_check_.reset();
    const bool running = response->success && same_plan(response->plan, sent_plan_);
    if (running) {
      not_running_since_.reset();
    } else if (!not_running_since_) {
      not_running_since_ = now;
    } else if (now - not_running_since_.value() > kNotRunningTolerance) {
      not_running_since_.reset();
      return true;
    }
  } else if (plan_check_ && now - plan_check_->sent > 2s) {
    // Without answer: asked again (no answer is no evidence either way)
    plan_check_client_->remove_pending_request(plan_check_->future);
    plan_check_.reset();
  }
  if (!plan_check_ && now - last_plan_check_ > 1s && plan_check_client_->service_is_ready()) {
    plan_check_ = PlanCheck{plan_check_client_->async_send_request(
        std::make_shared<plansys2_msgs::srv::GetPlan::Request>()), now};
    last_plan_check_ = now;
  }
  return false;
}

ExecutorClient::Recovery
ExecutorClient::recover_result(GoalHandleExecutePlan::WrappedResult & result)
{
  // Asked without waiting, as plan_not_running: the answer is read in a later call
  GoalHandleExecutePlan::SharedPtr goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal = goal_handler_;
  }
  if (!goal) {
    return Recovery::Lost;
  }
  if (!result_check_client_) {
    result_check_client_ = node_->create_client<GetResult>("execute_plan/_action/get_result");
  }
  const auto now = std::chrono::steady_clock::now();
  if (result_check_ && result_check_->future.wait_for(0s) == std::future_status::ready) {
    auto response = result_check_->future.get();
    result_check_.reset();
    result_attempts_ = 0;
    switch (response->status) {
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        result.code = rclcpp_action::ResultCode::SUCCEEDED;
        break;
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        result.code = rclcpp_action::ResultCode::ABORTED;
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        result.code = rclcpp_action::ResultCode::CANCELED;
        break;
      default:
        return Recovery::Lost;  // Unknown to it: a restarted executor
    }
    result.goal_id = goal->get_goal_id();
    result.result = std::make_shared<ExecutePlan::Result>(response->result);
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = result;
    feedback_ = ExecutePlan::Feedback();
    return Recovery::Recovered;
  }
  if (result_check_ && now - result_check_->sent > 2s) {
    // A goal that still runs answers only when it ends: not the plan sent any more
    result_check_client_->remove_pending_request(result_check_->future);
    result_check_.reset();
    if (++result_attempts_ >= kResultAttempts) {
      result_attempts_ = 0;
      return Recovery::Lost;
    }
  }
  if (!result_check_) {
    auto request = std::make_shared<GetResult::Request>();
    request->goal_id.uuid = goal->get_goal_id();
    result_check_ = ResultCheck{result_check_client_->async_send_request(request), now};
  }
  return Recovery::Pending;
}

bool
ExecutorClient::same_plan(const plansys2_msgs::msg::Plan & a, const plansys2_msgs::msg::Plan & b)
{
  if (a.items.size() != b.items.size()) {
    return false;
  }
  for (size_t i = 0; i < a.items.size(); i++) {
    if (a.items[i].action != b.items[i].action ||
      std::abs(a.items[i].time - b.items[i].time) > 1e-6)
    {
      return false;
    }
  }
  return true;
}

}  // namespace plansys2
