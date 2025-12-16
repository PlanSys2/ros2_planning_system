// Copyright (c) 2018 Intel Corporation
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

#ifndef PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_
#define PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "plansys2_bt_actions/BTUtils.hpp"
#include "plansys2_bt_actions/JSONUtils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
 *       It will re-initialize when halted.
 */
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A plansys2::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfig & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    if (!config().blackboard->get("node", node_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get 'node' from the blackboard");
    }
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    getInputOrBlackboard("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
      config().blackboard->get<std::chrono::milliseconds>("wait_for_service_timeout");

    // Timeout should be less than bt_loop_duration to be able to finish the current tick
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  /**
   * @brief Create instance of an action client
   * @param action_name Action name to create client for
   */
  bool createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());

    bool success_waiting = action_client_->wait_for_action_server(wait_for_service_timeout_);

    if (!success_waiting) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Timeout (%ld secs) waiting for \"%s\" action server",
        wait_for_service_timeout_.count() * 1000,
        action_name.c_str());
    }

    return success_waiting;
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>(
        "server_timeout",
        5000,
        "The amount of time to wait for a response from the action server, in milliseconds")
    };
    // The user defined ports are added to the basic ports
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard
   */
  virtual BT::NodeStatus on_tick()
  {
    return BT::NodeStatus::RUNNING;
  }

  /**
   * @brief Provides the opportunity for derived classes to log feedback, update the
   * goal, or cancel the goal
   * @param feedback The feedback received from the action server
   */
  virtual void on_feedback(
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    (void)feedback;
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    switch (state_) {
      case IDLE:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s IDLE", node_->get_name());
          assert((status() == BT::NodeStatus::IDLE));

          if (!createActionClient(action_name_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create action client");
            return BT::NodeStatus::FAILURE;
          }

          // User defined tick
          auto user_status = on_tick();
          if (user_status != BT::NodeStatus::RUNNING) {
            return user_status;
          }

          on_new_goal_received();

          state_ = GOAL_SENT;

          return BT::NodeStatus::RUNNING;
        }
        break;

      case GOAL_SENT:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_SENT", node_->get_name());
          if (future_goal_handle_) {
            auto elapsed =
              (node_->now() - goal_sent_ts_).template to_chrono<std::chrono::milliseconds>();
            if (!is_future_goal_handle_complete(elapsed)) {
              RCLCPP_ERROR(
                node_->get_logger(),
                "Goal was rejected by action server %s", action_name_.c_str());
              state_ = GOAL_FAILURE;
              future_goal_handle_.reset();
              return BT::NodeStatus::FAILURE;
            } else {
              state_ = GOAL_EXECUTING;
              return BT::NodeStatus::RUNNING;
            }
          } else {
            if ((node_->now() - goal_sent_ts_) > server_timeout_) {
              RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to send goal to action server %s", action_name_.c_str());
              state_ = GOAL_FAILURE;
              future_goal_handle_.reset();
              return BT::NodeStatus::FAILURE;
            } else {
              return BT::NodeStatus::RUNNING;
            }
          }
        }
        break;

      case GOAL_EXECUTING:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_EXECUTING", node_->get_name());
          auto goal_status = goal_handle_->get_status();
          auto user_status = on_tick();

          if (user_status != BT::NodeStatus::RUNNING) {
            cancel_goal();
            state_ = GOAL_CANCELLING;
            return user_status;
          }

          if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
            goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
          {
            on_new_goal_received();
            state_ = GOAL_SENT;
          }

          callback_group_executor_.spin_some();

          if (goal_result_available_) {
            state_ = GOAL_FINISHING;
          }

          return BT::NodeStatus::RUNNING;
        }
        break;

      case GOAL_FINISHING:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_FINISHING", node_->get_name());
          switch (result_.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              state_ = GOAL_FINISHED;
              return on_success();

            case rclcpp_action::ResultCode::ABORTED:
              state_ = GOAL_FINISHED;
              return on_aborted();

            case rclcpp_action::ResultCode::CANCELED:
              state_ = GOAL_FINISHED;
              return on_cancelled();

            default:
              throw std::logic_error("BtActionNode::Tick: invalid status value");
          }
        }
        break;

      case GOAL_CANCELLING:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_CANCELLING", node_->get_name());
          if (future_cancel_handle_.valid()) {
            state_ = GOAL_FINISHED;
            return BT::NodeStatus::SUCCESS;
          } else {
            RCLCPP_ERROR(
              node_->get_logger(),
              "Failed to cancel action server for %s", action_name_.c_str());
            state_ = GOAL_FAILURE;
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }
        break;

      case GOAL_FINISHED:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_FINISHED", node_->get_name());
          state_ = IDLE;
          return BT::NodeStatus::SUCCESS;
        }
        break;

      case GOAL_FAILURE:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_FAILURE", node_->get_name());
          state_ = IDLE;
          return BT::NodeStatus::FAILURE;
        }
        break;

      default:
        break;
    }

    goal_handle_.reset();
    return BT::NodeStatus::RUNNING;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */
  void halt() override
  {
    if (should_cancel_goal()) {
      cancel_goal();
    }

    resetStatus();
  }

protected:
  /**
   * @brief Function to cancel the current goal.
   */
  void cancel_goal()
  {
    if (goal_handle_) {
      future_cancel_handle_ = action_client_->async_cancel_goal(goal_handle_);
      if (callback_group_executor_.spin_until_future_complete(future_cancel_handle_,
          server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to cancel action server for %s", action_name_.c_str());
      }

      auto future_result = action_client_->async_get_result(goal_handle_);
      if (callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to get result for %s in node halt!", action_name_.c_str());
      }

      on_cancelled();
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "Cannot cancel goal for %s: goal handle is null", action_name_.c_str());
    }
  }

  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief Function to send a new goal to the action server
   */
  void on_new_goal_received()
  {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        on_feedback(feedback);
      };

    RCLCPP_INFO(
      node_->get_logger(), "Sending goal to action server %s", action_name_.c_str());

    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    goal_sent_ts_ = node_->now();
  }

/**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */
  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;

    // server has already timed out, no need to sleep
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    elapsed += timeout;

    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    return false;
  }

  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  future_goal_handle_;
  std::shared_future<typename ActionT::Impl::CancelGoalService::Response::SharedPtr>
  future_cancel_handle_;
  rclcpp::Time goal_sent_ts_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds max_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  static const int IDLE = 0;
  static const int GOAL_SENT = 1;
  static const int GOAL_EXECUTING = 2;
  static const int GOAL_FINISHING = 3;
  static const int GOAL_CANCELLING = 4;
  static const int GOAL_FINISHED = 5;
  static const int GOAL_FAILURE = 6;

  int state_ {IDLE};
};


}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_
