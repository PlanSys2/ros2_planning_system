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

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

using namespace std::chrono_literals;
using std::placeholders::_1;

ActionExecutorClient::ActionExecutorClient(const std::string & node_name)
: CascadeLifecycleNode(node_name),
  committed_(false)
{
  declare_parameter<std::string>("action_name", "");
  declare_parameter<std::vector<std::string>>(
    "specialized_arguments", std::vector<std::string>({}));

  period_ = std::chrono::milliseconds(200);
  double default_rate = 1.0 / std::chrono::duration<double>(period_).count();
  declare_parameter<double>("rate", default_rate);
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::NOT_READY;
  status_.status_stamp = now();
  status_.node_name = get_name();
  node_names_ = std::make_unique<NodeNames>(get_node_base_interface());
}

ActionExecutorClient::ActionExecutorClient(
  const std::string & node_name, const std::chrono::nanoseconds & rate)
: CascadeLifecycleNode(node_name),
  period_(rate), committed_(false)
{
  declare_parameter<std::string>("action_name", "");
  declare_parameter<std::vector<std::string>>(
    "specialized_arguments", std::vector<std::string>({}));

  double default_rate = 1.0 / std::chrono::duration<double>(period_).count();
  declare_parameter<double>("rate", default_rate);
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::NOT_READY;
  status_.status_stamp = now();
  status_.node_name = get_name();
  node_names_ = std::make_unique<NodeNames>(get_node_base_interface());
}

CallbackReturnT
ActionExecutorClient::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  status_pub_ = create_publisher<plansys2_msgs::msg::ActionPerformerStatus>(
    "performers_status", rclcpp::QoS(100).reliable());
  status_pub_->on_activate();

  hearbeat_pub_ = create_wall_timer(
    1s, [this]() {
      status_.status_stamp = now();
      status_pub_->publish(status_);
    });

  if (!get_parameter("action_name", action_managed_)) {
    RCLCPP_ERROR(get_logger(), "action_name parameter not set");
    status_.state = plansys2_msgs::msg::ActionPerformerStatus::FAILURE;
    status_.status_stamp = now();

    return CallbackReturnT::FAILURE;
  }

  get_parameter_or<std::vector<std::string>>(
    "specialized_arguments", specialized_arguments_, std::vector<std::string>({}));

  double rate = 1.0;  // If the parameter cannot be read
  get_parameter("rate", rate);

  period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / rate));

  action_hub_pub_ = create_publisher<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable());
  action_hub_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable(),
    std::bind(&ActionExecutorClient::action_hub_callback, this, _1));

  action_hub_pub_->on_activate();

  status_.state = plansys2_msgs::msg::ActionPerformerStatus::READY;
  status_.status_stamp = now();
  status_.action = action_managed_;
  status_.specialized_arguments = specialized_arguments_;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ActionExecutorClient::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::RUNNING;
  status_.status_stamp = now();
  start_time_ = now();
  timer_ = create_wall_timer(period_, std::bind(&ActionExecutorClient::do_work, this));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ActionExecutorClient::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::READY;
  status_.status_stamp = now();
  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

void
ActionExecutorClient::action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  switch (msg->type) {
    case plansys2_msgs::msg::ActionExecution::REQUEST:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        !committed_ && should_execute(msg->action, msg->arguments))
      {
        committed_ = true;
        committed_request_id_ = msg->request_id;
        requester_ = msg->node_id;
        commitment_timeout_ = create_wall_timer(
          5s, [this]() {
            RCLCPP_WARN(
              get_logger(), "No answer from %s to our response: available again",
              requester_.c_str());
            release_commitment();
          });
        send_response(msg);
      }
      break;
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        committed_ && msg->node_id == get_name() && msg->request_id == committed_request_id_)
      {
        current_arguments_ = msg->arguments;
        current_planned_duration_ = msg->planned_duration;
        current_request_id_ = msg->request_id;
        release_commitment();
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        // If activation failed, notify the executor so it can retry with another performer
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_ERROR(
            get_logger(),
            "Performer [%s] failed to activate for action [%s]. Notifying executor.",
            get_name(), action_managed_.c_str());
          finish(false, 0.0, "Activation failed for performer " + std::string(get_name()));
        } else {
          requester_misses_ = 0;
          requester_check_ = create_wall_timer(1s, [this]() {check_requester();});
        }
      }
      break;
    case plansys2_msgs::msg::ActionExecution::REJECT:
      if (committed_ && msg->node_id == get_name() && msg->request_id == committed_request_id_) {
        release_commitment();
      }
      break;
    case plansys2_msgs::msg::ActionExecution::CANCEL:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        msg->node_id == get_name() && msg->request_id == current_request_id_)
      {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
      break;
    case plansys2_msgs::msg::ActionExecution::UPDATE:
      // A new plan gives it another duration
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        msg->node_id == get_name() && msg->request_id == current_request_id_)
      {
        current_planned_duration_ = msg->planned_duration;
      }
      break;
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
    case plansys2_msgs::msg::ActionExecution::FINISH:
      break;
    default:
      RCLCPP_ERROR(
        get_logger(), "Msg %d type not recognized in %s executor performer",
        msg->type, get_name());
      break;
  }
}

void
ActionExecutorClient::release_commitment()
{
  committed_ = false;
  committed_request_id_.clear();
  if (commitment_timeout_) {
    commitment_timeout_->cancel();
    commitment_timeout_ = nullptr;
  }
}

void
ActionExecutorClient::check_requester()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    requester_check_->cancel();
    return;
  }

  // Names are fully qualified: compare the last part. A few misses in a row, as
  // discovery may lag, before stopping
  // Once rclcpp shuts down there is nothing to supervise
  auto found = node_names_->get(*get_node_graph_interface());
  if (!found) {
    return;
  }
  const auto & names = *found;
  bool alive = false;
  for (const auto & name : names) {
    if (name.substr(name.find_last_of('/') + 1) == requester_) {
      alive = true;
      break;
    }
  }
  requester_misses_ = alive ? 0 : requester_misses_ + 1;

  if (requester_misses_ >= 3) {
    RCLCPP_ERROR(
      get_logger(), "%s, which asked for %s, is gone: stopping the action",
      requester_.c_str(), action_managed_.c_str());
    requester_check_->cancel();
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }
}

bool
ActionExecutorClient::should_execute(
  const std::string & action, const std::vector<std::string> & args)
{
  // PDDL is case insensitive: Move performs move (#256)
  if (!parser::pddl::compare_str_case_insensitive(action, action_managed_)) {
    return false;
  }

  if (!specialized_arguments_.empty()) {
    // Fewer than its arguments is a prefix, as it is meant; more can never match
    if (specialized_arguments_.size() > args.size()) {
      RCLCPP_WARN(
        get_logger(), "%zu specialized arguments for an action of %zu: it never matches",
        specialized_arguments_.size(), args.size());
    }

    for (size_t i = 0; i < specialized_arguments_.size() && i < args.size(); i++) {
      if (specialized_arguments_[i] != "" && args[i] != "" &&
        !parser::pddl::compare_str_case_insensitive(specialized_arguments_[i], args[i]))
      {
        return false;
      }
    }
  }

  return true;
}

void
ActionExecutorClient::send_response(
  const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  plansys2_msgs::msg::ActionExecution msg_resp(*msg);
  msg_resp.type = plansys2_msgs::msg::ActionExecution::RESPONSE;
  msg_resp.node_id = get_name();

  action_hub_pub_->publish(msg_resp);
}

void
ActionExecutorClient::send_feedback(float completion, const std::string & status)
{
  plansys2_msgs::msg::ActionExecution msg_resp;
  msg_resp.type = plansys2_msgs::msg::ActionExecution::FEEDBACK;
  msg_resp.node_id = get_name();
  msg_resp.request_id = current_request_id_;
  msg_resp.action = action_managed_;
  msg_resp.arguments = current_arguments_;
  msg_resp.completion = completion;
  msg_resp.status = status;

  action_hub_pub_->publish(msg_resp);
}

void
ActionExecutorClient::finish(bool success, float completion, const std::string & status)
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  plansys2_msgs::msg::ActionExecution msg_resp;
  msg_resp.type = plansys2_msgs::msg::ActionExecution::FINISH;
  msg_resp.node_id = get_name();
  msg_resp.request_id = current_request_id_;
  msg_resp.action = action_managed_;
  msg_resp.arguments = current_arguments_;
  msg_resp.completion = completion;
  msg_resp.status = status;
  msg_resp.success = success;

  action_hub_pub_->publish(msg_resp);
}

}  // namespace plansys2
