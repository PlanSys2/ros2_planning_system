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

#ifndef PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_

#include <optional>
#include <string>
#include <memory>
#include <list>
#include <vector>

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

/**
 * @class plansys2::ExecutorClient
 * @brief Client interface for the plansys2 executor node.
 *
 * This class provides a client interface to interact with the ExecutorNode,
 * allowing applications to request plan execution, monitor progress,
 * cancel plans, and retrieve execution information.
 */
class ExecutorClient
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ClientGoalHandle<ExecutePlan>;

  /**
   * @brief Default constructor.
   */
  ExecutorClient();

  /**
   * @brief Constructor with custom node name.
   *
   * Creates a ROS node with the specified name and initializes service clients
   * and action clients to communicate with the executor node.
   *
   * @param[in] node_name Name for the ROS node.
   */
  explicit ExecutorClient(const std::string & node_name);

  /**
   * @brief Start the execution of a plan.
   *
   * @param[in] plan The plan to be executed.
   * @return true if the plan was successfully sent and accepted, false otherwise.
   */
  bool start_plan_execution(const plansys2_msgs::msg::Plan & plan);

  /**
   * @brief Execute the current plan and check its status.
   *
   * @return true if plan execution is still in progress, false if plan has finished
   *         (either successfully, cancelled, or failed).
   */
  bool execute_and_check_plan();

  /**
   * @brief Cancel the execution of the current plan.
   */
  void cancel_plan_execution();

  /**
   * @brief Get the ordered sub-goals of the current plan.
   *
   * Retrieves the ordered sub-goals derived from the current plan being executed.
   * These represent intermediate goals to be achieved during plan execution.
   *
   * @return std::vector<plansys2_msgs::msg::Tree> Vector of tree structures representing
   *         the ordered sub-goals.
   */
  std::vector<plansys2_msgs::msg::Tree> getOrderedSubGoals();

  /**
   * @brief Get the complete plan being executed.
   *
   * @return std::optional<plansys2_msgs::msg::Plan> The complete plan if available,
   *         empty if no plan is currently being executed or if an error occurred.
   */
  std::optional<plansys2_msgs::msg::Plan> get_plan();

  /**
   * @brief Get the remaining portion of the plan to be executed.
   *
   * @return std::optional<plansys2_msgs::msg::Plan> The remaining plan if available,
   *         empty if no remaining plan is available or if an error occurred.
   */
  std::optional<plansys2_msgs::msg::Plan> get_remaining_plan();

  /**
   * @brief Get the current feedback from plan execution.
   *
   * @return ExecutePlan::Feedback The current feedback message from the executor.
   */
  ExecutePlan::Feedback getFeedBack() {return feedback_;}

  /**
   * @brief Get the final result of plan execution.
   *
   * @return std::optional<ExecutePlan::Result> The result of the plan execution if available,
   *         empty if no result is available or if an error occurred.
   */
  std::optional<ExecutePlan::Result> getResult();

private:
  /**
   * @brief Callback for receiving action results.
   *
   * @param[in] result The result of the plan execution.
   */
  void result_callback(const GoalHandleExecutePlan::WrappedResult & result);

  /**
   * @brief Callback for receiving action feedback.
   *
   * @param[in] goal_handle Handle to the current goal.
   * @param[in] feedback Feedback from the plan execution.
   */
  void feedback_callback(
    GoalHandleExecutePlan::SharedPtr goal_handle,
    const std::shared_ptr<const ExecutePlan::Feedback> feedback);

  /**
   * @brief Process a new goal request.
   *
   * @param[in] plan The plan to be executed.
   * @return true if the goal was successfully sent and accepted, false otherwise.
   */
  bool on_new_goal_received(const plansys2_msgs::msg::Plan & plan);

  /**
   * @brief Check if the current goal should be cancelled.
   *
   * @return true if the goal should be cancelled, false otherwise.
   */
  bool should_cancel_goal();

  /**
   * @brief Create the action client for plan execution.
   *
   * Initializes the action client and waits for the action server to be available.
   */
  void createActionClient();

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<ExecutePlan>::SharedPtr action_client_;
  rclcpp::Client<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_remaining_plan_client_;

  ExecutePlan::Feedback feedback_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::SharedPtr goal_handler_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult result_;

  bool goal_result_available_{false};

  bool executing_plan_{false};
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
