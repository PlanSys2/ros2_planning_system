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

#include <algorithm>
#include <map>
#include <string>
#include <memory>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_executor/ActionExecutor.hpp"

namespace plansys2
{

using namespace std::chrono_literals;

namespace
{
// Looking for a performer this long fails the action
constexpr double kDealingTimeout = 30.0;
// A performer that fails to activate is replaced this many times before failing the action
constexpr int kMaxActivationFailures = 3;
// Checks in a row without the confirmed performer before failing the action
constexpr int kMaxPerformerMisses = 3;
}  // namespace

ActionHub::Ptr
ActionHub::get(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  static std::mutex registry_mutex;
  static std::map<const rclcpp_lifecycle::LifecycleNode *, std::weak_ptr<ActionHub>> registry;

  std::lock_guard<std::mutex> lock(registry_mutex);
  for (auto it = registry.begin(); it != registry.end(); ) {
    it = it->second.expired() ? registry.erase(it) : std::next(it);
  }
  auto & entry = registry[node.get()];
  auto hub = entry.lock();
  if (!hub) {
    hub = std::make_shared<ActionHub>(node);
    hub->subscribe(node);
    entry = hub;
  }
  return hub;
}

ActionHub::ActionHub(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  pub_ = node->create_publisher<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable());
  // Active for the whole life of the hub: a late answer must still get its REJECT
  pub_->on_activate();
}

void
ActionHub::subscribe(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  sub_ = node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable(),
    [hub = weak_from_this()](plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      if (auto alive = hub.lock()) {
        alive->dispatch(msg);
      }
    });
}

void
ActionHub::publish(const plansys2_msgs::msg::ActionExecution & msg)
{
  pub_->publish(msg);
}

void
ActionHub::add(const std::weak_ptr<ActionExecutor> & executor)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto alive = executor.lock()) {
    executors_.emplace_back(alive.get(), executor);
  }
}

void
ActionHub::remove(const ActionExecutor * executor)
{
  std::lock_guard<std::mutex> lock(mutex_);
  executors_.erase(
    std::remove_if(
      executors_.begin(), executors_.end(),
      [executor](const auto & entry) {return entry.first == executor;}),
    executors_.end());
}

void
ActionHub::dispatch(plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  // Outside the lock: an executor may add or remove executors while it handles the message
  std::vector<std::shared_ptr<ActionExecutor>> alive;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = executors_.begin();
    while (it != executors_.end()) {
      if (auto executor = it->second.lock()) {
        alive.push_back(executor);
        ++it;
      } else {
        it = executors_.erase(it);
      }
    }
  }
  for (const auto & executor : alive) {
    executor->action_hub_callback(msg);
  }
}

ActionExecutor::ActionExecutor(
  const std::string & action,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: node_(node), state_(IDLE), completion_(0.0)
{
  hub_ = ActionHub::get(node_);
  node_names_ = std::make_unique<NodeNames>(node_->get_node_base_interface());

  state_time_ = node_->now();

  action_ = action;
  action_name_ = get_name(action);
  action_params_ = get_params(action);
  start_execution_ = node_->now();
  state_time_ = start_execution_;
}

ActionExecutor::~ActionExecutor()
{
  clean_up();
}

void
ActionExecutor::clean_up()
{
  if (hub_) {
    hub_->remove(this);
  }
  if (waiting_timer_) {
    waiting_timer_->cancel();
  }
  if (performer_check_) {
    performer_check_->cancel();
  }
}

void
ActionExecutor::action_hub_callback(plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_ = msg;

  // A performer of our action is free now: ask again at once, not at the next retry. Rejected,
  // it ignored our request while it waited for the answer to another one
  const bool freed = msg->type == plansys2_msgs::msg::ActionExecution::FINISH ||
    msg->type == plansys2_msgs::msg::ActionExecution::REJECT;
  if (state_ == DEALING && freed && msg->request_id != request_id_ &&
    parser::pddl::compare_str_case_insensitive(msg->action, action_name_))
  {
    request_for_performers();
    return;
  }

  // Only the messages of our own request: not those of another executor of the same action
  if (msg->request_id != request_id_ ||
    !parser::pddl::compare_str_case_insensitive(msg->action, action_name_) ||
    msg->arguments != action_params_)
  {
    return;
  }

  switch (msg->type) {
    case plansys2_msgs::msg::ActionExecution::REQUEST:
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
    case plansys2_msgs::msg::ActionExecution::REJECT:
    case plansys2_msgs::msg::ActionExecution::CANCEL:
    case plansys2_msgs::msg::ActionExecution::UPDATE:
      // These cases have no meaning requester
      break;
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
      if (state_ == DEALING) {
        confirm_performer(msg->node_id);
        current_performer_id_ = msg->node_id;
        state_ = RUNNING;
        if (waiting_timer_) {
          waiting_timer_->cancel();
        }
        waiting_timer_ = nullptr;
        start_execution_ = node_->now();
        state_time_ = node_->now();
        performer_misses_ = 0;
        performer_check_ = create_timer(&ActionExecutor::check_performer);
      } else {
        // Someone else answered first, or the action is over: it must not wait for us
        reject_performer(msg->node_id);
      }
      break;
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
      if (state_ != RUNNING || msg->node_id != current_performer_id_) {
        return;
      }
      feedback_ = msg->status;
      completion_ = msg->completion;
      state_time_ = node_->now();

      break;
    case plansys2_msgs::msg::ActionExecution::FINISH:
      // Late ones, e.g. after a cancel, do not change how the action ended
      if (state_ != RUNNING || msg->node_id != current_performer_id_) {
        return;
      }
      feedback_ = msg->status;
      completion_ = msg->completion;
      state_time_ = node_->now();
      if (performer_check_) {
        performer_check_->cancel();
      }

      if (msg->success) {
        state_ = SUCCESS;
      } else if (msg->completion == 0.0 && ++activation_failures_ <= kMaxActivationFailures) {
        // Performer failed before doing any work (likely activation failure): another one
        RCLCPP_WARN(
          node_->get_logger(),
          "Performer [%s] for action [%s] failed before starting. Retrying (%d of %d).",
          current_performer_id_.c_str(), action_.c_str(), activation_failures_,
          kMaxActivationFailures);
        start_dealing();
      } else {
        state_ = FAILURE;
      }
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Msg %d type not recognized in %s executor requester",
        msg->type, action_.c_str());
      break;
  }
}

void
ActionExecutor::confirm_performer(const std::string & node_id)
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CONFIRM;
  msg.node_id = node_id;
  msg.request_id = request_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;
  msg.planned_duration = planned_duration_;

  hub_->publish(msg);
}

void
ActionExecutor::reject_performer(const std::string & node_id)
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REJECT;
  msg.node_id = node_id;
  msg.request_id = request_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;

  hub_->publish(msg);
}

void
ActionExecutor::request_for_performers()
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REQUEST;
  msg.node_id = node_->get_name();
  msg.request_id = request_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;
  msg.planned_duration = planned_duration_;

  hub_->publish(msg);
}

void
ActionExecutor::start_dealing()
{
  static std::atomic<uint64_t> requests {0};
  // Unique also against requests of a previous run of the executor
  request_id_ = std::string(node_->get_node_base_interface()->get_fully_qualified_name()) + ":" +
    std::to_string(node_->now().nanoseconds()) + ":" + std::to_string(requests++);

  current_performer_id_ = "";
  failure_reason_.clear();
  state_ = DEALING;
  state_time_ = node_->now();
  completion_ = 0.0;
  feedback_ = "";

  request_for_performers();
  if (waiting_timer_) {
    waiting_timer_->cancel();
  }
  waiting_timer_ = create_timer(&ActionExecutor::wait_timeout);
}

rclcpp::TimerBase::SharedPtr
ActionExecutor::create_timer(void (ActionExecutor::* callback)())
{
  return node_->create_wall_timer(
    1s, [self = self_, callback]() {
      if (auto executor = self->lock()) {
        ((*executor).*callback)();
      }
    });
}

BT::NodeStatus
ActionExecutor::get_status()
{
  switch (state_) {
    case IDLE:
      return BT::NodeStatus::IDLE;
      break;
    case DEALING:
    case RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    case SUCCESS:
      return BT::NodeStatus::SUCCESS;
      break;
    case FAILURE:
    // A BT node must not return IDLE once it started: BT.CPP throws
    case CANCELLED:
      return BT::NodeStatus::FAILURE;
      break;
    default:
      return BT::NodeStatus::IDLE;
      break;
  }
}

bool
ActionExecutor::is_finished()
{
  return state_ == SUCCESS || state_ == FAILURE;
}

BT::NodeStatus
ActionExecutor::tick(const rclcpp::Time & now)
{
  (void)now;
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == IDLE) {
    activation_failures_ = 0;
    start_dealing();
  }

  return get_status();
}

void
ActionExecutor::update_planned_duration(double duration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planned_duration_ = duration;
  if (state_ != RUNNING) {
    return;
  }
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::UPDATE;
  msg.node_id = current_performer_id_;
  msg.request_id = request_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;
  msg.planned_duration = duration;
  hub_->publish(msg);
}

void
ActionExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = CANCELLED;
  // Stop asking for performers (#436)
  if (waiting_timer_) {
    waiting_timer_->cancel();
  }
  if (performer_check_) {
    performer_check_->cancel();
  }
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CANCEL;
  msg.node_id = current_performer_id_;
  msg.request_id = request_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;

  hub_->publish(msg);
}

std::string
ActionExecutor::get_name(const std::string & action_expr)
{
  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  return working_action_expr.substr(0, delim);
}

std::vector<std::string>
ActionExecutor::get_params(const std::string & action_expr)
{
  std::vector<std::string> ret;

  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
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

void
ActionExecutor::wait_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != DEALING) {
    return;
  }
  // Here and not in tick: it holds even if the BT is no longer ticked (#365)
  if ((node_->now() - state_time_).seconds() > kDealingTimeout) {
    RCLCPP_ERROR(
      node_->get_logger(), "Aborting %s. Timeout after requesting for %.0f seconds",
      action_.c_str(), kDealingTimeout);
    // The reason the controller and the trace tell
    failure_reason_ = "no performer took it in " +
      std::to_string(static_cast<int>(kDealingTimeout)) + " s";
    feedback_ = failure_reason_;
    state_ = FAILURE;
    waiting_timer_->cancel();
    return;
  }
  RCLCPP_WARN(node_->get_logger(), "No action performer for %s. retrying", action_.c_str());
  request_for_performers();
}

void
ActionExecutor::check_performer()
{
  // Once rclcpp shuts down there is nothing to supervise
  auto found = node_names_->get(*node_->get_node_graph_interface());
  if (!found) {
    return;
  }
  const auto & names = *found;

  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != RUNNING) {
    performer_check_->cancel();
    return;
  }

  // Names are fully qualified: compare the last part. A few misses in a row, as discovery
  // may lag. An action may run long without feedback, but not without its performer
  bool alive = false;
  for (const auto & name : names) {
    if (name.substr(name.find_last_of('/') + 1) == current_performer_id_) {
      alive = true;
      break;
    }
  }
  performer_misses_ = alive ? 0 : performer_misses_ + 1;
  if (performer_misses_ >= kMaxPerformerMisses) {
    RCLCPP_ERROR(
      node_->get_logger(), "Performer [%s] of %s is gone: the action fails",
      current_performer_id_.c_str(), action_.c_str());
    // Its last feedback would be told as the reason ("running"): the reason is that it is gone
    failure_reason_ = "its performer " + current_performer_id_ + " is gone";
    feedback_ = failure_reason_;
    state_ = FAILURE;
    performer_check_->cancel();
  }
}

}  // namespace plansys2
