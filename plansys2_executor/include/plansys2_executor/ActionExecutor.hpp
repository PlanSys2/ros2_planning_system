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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <vector>

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_core/Action.hpp"
#include "plansys2_core/NodeNames.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/FactChange.hpp"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

class ActionExecutor;

/**
 * @class plansys2::ActionHub
 * @brief The actions_hub publisher and subscription of a node, shared by its action executors.
 *
 * Each executor had its own: a plan of N actions made N publishers and N subscriptions, and
 * every message was delivered N times. The hub delivers each one to the executors alive, which
 * keep only those of their request.
 */
class ActionHub : public std::enable_shared_from_this<ActionHub>
{
public:
  using Ptr = std::shared_ptr<ActionHub>;

  /**
   * @brief The hub of a node, created the first time and kept while someone holds it.
   *
   * @param[in] node The lifecycle node.
   * @return The hub.
   */
  static Ptr get(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  explicit ActionHub(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Publish a message on actions_hub.
   * @param[in] msg The message.
   */
  void publish(const plansys2_msgs::msg::ActionExecution & msg);

  /**
   * @brief Deliver the messages of actions_hub to an executor, until it is removed or destroyed.
   * @param[in] executor The executor.
   */
  void add(const std::weak_ptr<ActionExecutor> & executor);

  /**
   * @brief Stop delivering messages to an executor.
   * @param[in] executor The executor.
   */
  void remove(const ActionExecutor * executor);

private:
  // The subscription holds the hub weakly: it may run while the hub is destroyed
  void subscribe(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  void dispatch(plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecution>::SharedPtr pub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<std::pair<const ActionExecutor *, std::weak_ptr<ActionExecutor>>> executors_;
};

/**
 * @class plansys2::ActionExecutor
 * @brief Class that manages the execution of a PDDL action by communicating with action performers.
 *
 * This class handles the lifecycle of action execution by sending requests to appropriate
 * action performers, monitoring their progress, and reporting completion status.
 * It maintains state information about the execution and communicates over the actions_hub topic.
 */
class ActionExecutor
{
  // It delivers the messages of actions_hub to action_hub_callback
  friend class ActionHub;

public:
  /**
   * @brief Status enumeration representing the action execution state.
   */
  enum Status
  {
    IDLE,
    DEALING,
    RUNNING,
    SUCCESS,
    FAILURE,
    CANCELLED
  };

  using Ptr = std::shared_ptr<ActionExecutor>;

  /**
   * @brief Factory method to create a shared pointer to an ActionExecutor.
   *
   * @param[in] action The action expression to execute (in PDDL format).
   * @param[in] node The lifecycle node used for ROS communications.
   * @return Shared pointer to the newly created ActionExecutor.
   */
  static Ptr make_shared(
    const std::string & action,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    auto ret = std::make_shared<ActionExecutor>(action, node);
    *ret->self_ = ret;
    ret->hub_->add(ret);
    return ret;
  }

  /**
   * @brief Constructor for the ActionExecutor.
   *
   * @param[in] action The action expression to execute (in PDDL format).
   * @param[in] node The lifecycle node used for ROS communications.
   */
  explicit ActionExecutor(
    const std::string & action, rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Destructor for the ActionExecutor.
   */
  ~ActionExecutor();

  /**
   * @brief Process one execution cycle for the action.
   *
   * Advances the state machine based on the current state:
   * - IDLE → DEALING: sends a request for performers
   * - DEALING: checks for timeout
   * - RUNNING: allows feedback to be processed
   *
   * @param[in] now Current ROS time.
   * @return BT::NodeStatus BehaviorTree status corresponding to the current execution state.
   */
  BT::NodeStatus tick(const rclcpp::Time & now);

  /**
   * @brief Cancel the action execution.
   *
   * Sets the state to CANCELLED and sends a cancellation message to the performer.
   */
  void cancel();

  /**
   * @brief Get the current status as a BehaviorTree node status.
   *
   * @return BT::NodeStatus The current status as a BehaviorTree node status.
   */
  BT::NodeStatus get_status();

  /**
   * @brief Check if the action execution has finished.
   *
   * @return true if the action has succeeded or failed, false otherwise.
   */
  bool is_finished();

  // Methods for debug

  /**
   * @brief Get the internal execution status.
   *
   * @return Status The current status enum value.
   */
  Status get_internal_status() const {return state_.load();}

  /**
   * @brief Set the internal execution status.
   *
   * @param[in] state The new status to set.
   */
  void set_internal_status(Status state) {state_ = state;}

  /**
   * @brief Get the name of the action being executed.
   *
   * @return std::string The action name.
   */
  std::string get_action_name() const {return action_name_;}

  /**
   * @brief Get the parameters of the action being executed.
   *
   * @return std::vector<std::string> Vector of parameter strings.
   */
  std::vector<std::string> get_action_params() const {return action_params_;}

  /**
   * @brief Get the time when the action execution started.
   *
   * @return rclcpp::Time The start time of the execution.
   */
  rclcpp::Time get_start_time() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return start_execution_;
  }

  /**
   * @brief Get the current ROS time from the node.
   *
   * @return rclcpp::Time Current ROS time.
   */
  rclcpp::Time get_current_time() const {return node_->now();}

  /**
   * @brief Get the time of the last state change.
   *
   * @return ROS time of the last state change.
   */
  rclcpp::Time get_status_time() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_time_;
  }

  /**
   * @brief Get the current feedback from the action performer.
   *
   * @return std::string containing the feedback message.
   */
  std::string get_feedback() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_;
  }

  /**
   * @brief The performer that took the action, empty until one does.
   */
  std::string get_performer() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_performer_id_;
  }

  /**
   * @brief Why the executor failed the action, when it was not its performer that said so
   * (no performer took it, its performer is gone); empty otherwise.
   */
  std::string get_failure_reason() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return failure_reason_;
  }

  /**
   * @brief Get the completion percentage of the action.
   *
   * @return float Completion percentage (0.0 to 1.0).
   */
  float get_completion() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return completion_;
  }

  /**
   * @brief The duration the plan gives the action, sent to its performer.
   * @param[in] duration In seconds; 0 for a non-durative action.
   */
  void set_planned_duration(double duration) {planned_duration_ = duration;}

  /**
   * @brief Another duration for the running action, from its start: its performer is told.
   * @param[in] duration In seconds.
   */
  void update_planned_duration(double duration);

  /**
   * @brief The duration the plan gives the action.
   * @return In seconds.
   */
  double get_planned_duration() const {return planned_duration_;}

  /**
   * @brief Clean up resources used by the executor.
   */
  void clean_up();

  plansys2_msgs::msg::ActionExecution::SharedPtr last_msg_;

protected:
  /**
   * @brief Process messages from the actions hub.
   *
   * Handles different message types:
   * - RESPONSE: From performers accepting the request
   * - FEEDBACK: Updates on execution progress
   * - FINISH: Final completion status
   *
   * @param[in] msg The action execution message received.
   */
  void action_hub_callback(plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  /**
   * @brief Send a request for performers to execute this action.
   *
   * Publishes a REQUEST message to the actions_hub topic.
   */
  void request_for_performers();

  /**
   * @brief Confirm a performer for this action.
   *
   * Sends a CONFIRM message to the selected performer.
   *
   * @param[in] node_id ID of the performer to confirm.
   */
  void confirm_performer(const std::string & node_id);

  /**
   * @brief Reject a performer for this action.
   *
   * Sends a REJECT message to the performer.
   *
   * @param[in] node_id ID of the performer to reject.
   */
  void reject_performer(const std::string & node_id);

  /**
   * @brief Extract the action name from an action expression.
   *
   * Parses a PDDL action expression to extract just the action name.
   *
   * @param[in] action_expr The action expression to parse.
   * @return std::string The extracted action name.
   */
  std::string get_name(const std::string & action_expr);

  /**
   * @brief Extract parameters from an action expression.
   *
   * Parses a PDDL action expression to extract the parameter values.
   *
   * @param[in] action_expr The action expression to parse.
   * @return std::vector<std::string> Vector of extracted parameter strings.
   */
  std::vector<std::string> get_params(const std::string & action_expr);

  /**
   * @brief Timeout handler for performer requests.
   *
   * Called when no performer responds in time, logs a warning and retries.
   */
  void wait_timeout();

  // The confirmed performer is still there; it fails the action if it is gone
  void check_performer();

  // Starts asking for performers, with a new request id
  void start_dealing();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Callbacks reach the executor through this, outliving it: one running while the
  // executor is destroyed finds it gone instead of using freed memory
  std::shared_ptr<std::weak_ptr<ActionExecutor>> self_ =
    std::make_shared<std::weak_ptr<ActionExecutor>>();
  // A timer whose callback runs on this executor, only while it exists
  rclcpp::TimerBase::SharedPtr create_timer(void (ActionExecutor::* callback)());

  std::atomic<Status> state_;
  // Hub messages, BT ticks, cancel and the getters come from different threads
  mutable std::mutex mutex_;
  std::string request_id_;
  int activation_failures_ {0};
  int performer_misses_ {0};
  rclcpp::TimerBase::SharedPtr performer_check_;
  std::unique_ptr<NodeNames> node_names_;
  rclcpp::Time state_time_;
  rclcpp::Time start_execution_;

  std::string action_;
  std::string action_name_;
  std::string current_performer_id_;
  std::vector<std::string> action_params_;

  std::string feedback_;
  std::string failure_reason_;
  float completion_;
  double planned_duration_ {0.0};

  ActionHub::Ptr hub_;

  rclcpp::TimerBase::SharedPtr waiting_timer_;
};

/**
 * @brief Structure that holds execution information for an action.
 *
 * This structure maintains the runtime state of an action's execution,
 * including timing information, effect application status, and error details.
 */
struct ActionExecutionInfo
{
  plansys2_msgs::msg::PlanItem plan_item;
  std::shared_ptr<ActionExecutor> action_executor = {nullptr};
  bool at_start_effects_applied = {false};
  bool at_end_effects_applied = {false};
  rclcpp::Time at_start_effects_applied_time;
  rclcpp::Time at_end_effects_applied_time;
  // What the at start effects changed, to take it back if the action does not end
  std::vector<plansys2::FactChange> at_start_changes;
  ActionVariant action_info;
  std::string execution_error_info;
  double duration = 0.0;
  double duration_overrun_percentage = -1.0;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
