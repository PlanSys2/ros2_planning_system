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

#include <chrono>
#include <future>
#include <optional>
#include <string>
#include <memory>
#include <mutex>
#include <list>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/ServiceCall.hpp"
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
   * @brief Creates the client on a node it does not own: the node of a PlanSys2 component or of
   * an application. It creates no node of its own, and does not wait for the executor here.
   */
  explicit ExecutorClient(ClientNode::Interfaces node);

  template<typename NodeT>
  explicit ExecutorClient(const std::shared_ptr<NodeT> & node)
  : ExecutorClient(ClientNode::Interfaces(*node)) {}

  /**
   * @brief Waits until its component is active, as every call does: false if ROS shuts down.
   */
  bool wait_active() {return active_->wait();}

  /**
   * @brief While interrupted, its calls give up at once instead of waiting: for the owner that
   * stops a thread which may be in a call.
   */
  void interrupt(bool interrupted) {node_->interrupt(interrupted);}

  /**
   * @brief How long a call waits for its component to be active before it gives up (zero, the
   * default: as long as it takes).
   */
  void set_max_wait(std::chrono::milliseconds max_wait) {active_->set_max_wait(max_wait);}

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
   * @brief The problem to plan again from while a plan runs: the one of now, before the at start
   *        effects of the actions that run. A plan made from it has them again: those of the new
   *        plan that are running go on.
   *
   * @return The problem, empty if the executor cannot give it.
   */
  std::optional<std::string> get_replanning_problem();

  /**
   * @brief Get the current feedback from plan execution.
   *
   * @return ExecutePlan::Feedback The current feedback message from the executor.
   */
  ExecutePlan::Feedback getFeedBack()
  {
    node_->serve();
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_;
  }

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
  rclcpp_action::Client<ExecutePlan>::SharedPtr make_action_client(ClientNode::Interfaces node);
  // Sent this many times at most: an answer lost, or rejected while the executor is not active
  static constexpr int kGoalAttempts = 3;
  // Whether the executor has not run the plan sent for kNotRunningTolerance (restarted, lost)
  bool plan_not_running();
  static bool same_plan(const plansys2_msgs::msg::Plan & a, const plansys2_msgs::msg::Plan & b);
  static constexpr std::chrono::seconds kNotRunningTolerance {3};
  struct PlanCheck
  {
    rclcpp::Client<plansys2_msgs::srv::GetPlan>::FutureAndRequestId future;
    std::chrono::steady_clock::time_point sent;
  };
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr plan_check_client_;
  std::optional<PlanCheck> plan_check_;
  std::chrono::steady_clock::time_point last_plan_check_;
  std::optional<std::chrono::steady_clock::time_point> not_running_since_;
  plansys2_msgs::msg::Plan sent_plan_;

  // Not running: its result is asked for again (an answer lost on the way), before taking the
  // plan as lost. The executor keeps the result of an ended goal for a while
  enum class Recovery {Pending, Recovered, Lost};
  Recovery recover_result(GoalHandleExecutePlan::WrappedResult & result);
  static constexpr int kResultAttempts = 2;
  using GetResult = ExecutePlan::Impl::GetResultService;
  struct ResultCheck
  {
    rclcpp::Client<GetResult>::FutureAndRequestId future;
    std::chrono::steady_clock::time_point sent;
  };
  rclcpp::Client<GetResult>::SharedPtr result_check_client_;
  std::optional<ResultCheck> result_check_;
  int result_attempts_ {0};
  bool recovering_ {false};

  /**
   * @brief Check if the current goal should be cancelled.
   *
   * @return true if the goal should be cancelled, false otherwise.
   */
  bool should_cancel_goal();

  std::shared_ptr<ClientNode> node_;
  // Its component: every call waits until it is active
  std::shared_ptr<ActiveComponent> active_;

  rclcpp_action::Client<ExecutePlan>::SharedPtr action_client_;
  ServiceClient<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_client_;
  ServiceClient<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_client_;
  ServiceClient<plansys2_msgs::srv::GetPlan>::SharedPtr get_remaining_plan_client_;
  ServiceClient<plansys2_msgs::srv::GetProblem>::SharedPtr get_replanning_problem_client_;

  // Feedback, result and goal come while the client serves its group, maybe in another thread
  std::mutex mutex_;
  ExecutePlan::Feedback feedback_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::SharedPtr goal_handler_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult result_;

  bool goal_result_available_{false};

  bool executing_plan_{false};
  // Since when the executor's action server has been missing, while executing (#446)
  std::optional<std::chrono::steady_clock::time_point> server_lost_since_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
