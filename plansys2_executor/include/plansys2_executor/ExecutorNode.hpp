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

#ifndef PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <optional>
#include <map>
#include <tuple>
#include <atomic>
#include <list>
#include <mutex>
#include <thread>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_executor/ConstraintMonitor.hpp"
#include "plansys2_executor/ExecutionTrace.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pluginlib/class_loader.hpp"

namespace plansys2
{

/**
 * @brief Contains information about a behavior tree for plan execution.
 *
 * This structure stores the behavior tree, its blackboard, and the BT builder
 * that created it. It's used to manage the execution of a plan through a BT.
 */
struct TreeInfo
{
  using Ptr = std::shared_ptr<TreeInfo>;
  BT::Tree tree;
  BT::Blackboard::Ptr blackboard;
  std::shared_ptr<plansys2::BTBuilder> bt_builder;
};

/**
 * @brief Contains runtime information for plan execution.
 *
 * This structure maintains the state of plan execution, including the remaining
 * and complete plans, ordered sub-goals, execution status of actions, and
 * the current behavior tree being executed.
 */
struct PlanRuntineInfo
{
  plansys2_msgs::msg::Plan remaining_plan;
  plansys2_msgs::msg::Plan complete_plan;
  std::vector<plansys2_msgs::msg::Tree> ordered_sub_goals;
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map;
  TreeInfo::Ptr current_tree;
  // When its tree started, and whether it is done and the plan waits for its end (PDDL+)
  rclcpp::Time plan_start {0, 0, RCL_ROS_TIME};
  bool tree_done = false;
  // PDDL3: the hard constraints of the domain and the problem, followed while it runs
  std::shared_ptr<ConstraintMonitor> constraints;
  // When the plan started in the time of the constraints: a plan replaced while running goes on
  // with the history and the times of the first one
  double constraints_offset = 0.0;
};

/**
 * @class plansys2::ExecutorNode
 * @brief ROS2 Lifecycle node that manages plan execution.
 *
 * This node receives plans to execute, transforms them into behavior trees,
 * and monitors their execution. It also provides services to retrieve information
 * about the current execution state and supports replanning when necessary.
 */
class ExecutorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor for the ExecutorNode.
   */
  explicit ExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the ExecutorNode.
   */
  virtual ~ExecutorNode();

  /**
   * @brief Configures the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if configuration is successful, FAILURE otherwise.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if activation is successful, FAILURE otherwise.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if deactivation is successful, FAILURE otherwise.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if cleanup is successful, FAILURE otherwise.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if shutdown is successful, FAILURE otherwise.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if error handling is successful, FAILURE otherwise.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Service callback to get ordered sub-goals derived from the plan.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the ordered sub-goals and success status.
   */
  void get_ordered_sub_goals_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response);

  /**
   * @brief Service callback to get the complete plan being executed.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the complete plan and success status.
   */
  void get_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

  /**
   * @brief Service callback to get the remaining plan yet to be executed.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Empty service request.
   * @param[out] response Service response containing the remaining plan and success status.
   */
  void get_remaining_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

  // The problem to plan again from while a plan runs: before the at start effects of what runs.
  // Answered later, by the execution thread
  void get_replanning_problem_service_callback(
    std::shared_ptr<rclcpp::Service<plansys2_msgs::srv::GetProblem>> service,
    std::shared_ptr<rmw_request_id_t> request_header);

  /**
   * @brief Callback for domain_expert/domain. Cancels any plan currently
   * executing when the domain changes at runtime, since it may no longer
   * match the domain the plan was computed against. Skips the initial
   * transient_local replay of the domain the node already started with.
   *
   * @param[in] msg The new domain content.
   */
  void domain_topic_callback(const std_msgs::msg::String::SharedPtr msg);

  // Decimals of the times in the action ids, from action_time_precision
  int action_time_precision();

protected:
  /**
   * @brief Extracts ordered sub-goals from a plan.
   *
   * @param[in,out] runtime_info Runtime information to update with ordered sub-goals.
   */
  void get_ordered_subgoals(PlanRuntineInfo & runtime_info);

  /**
   * @brief Constructs feedback information about action execution.
   *
   * @param[in] action_map Map of action IDs to execution information.
   * @return Vector of action execution info messages.
   */
  std::vector<plansys2_msgs::msg::ActionExecutionInfo> get_feedback_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  /**
   * @brief Prints execution information to stderr for debugging.
   *
   * @param[in] exec_info Map of action IDs to execution information.
   */
  void print_execution_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info);

  /**
   * @brief Callback for handling new goal requests.
   *
   * @param[in] uuid Goal UUID.
   * @param[in] goal Goal containing the plan to execute.
   * @return Goal response indicating acceptance.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal);

  /**
   * @brief Callback for handling cancellation requests.
   *
   * @param[in] goal_handle Handle to the goal being canceled.
   * @return Cancel response indicating acceptance or rejection.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * @brief Callback for handling accepted goals.
   *
   * @param[in] goal_handle Handle to the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * @brief Removes completed goal handlers from the list.
   */
  void purge_handlers_list();

  /**
   * @brief Main execution cycle that runs in a separate thread.
   *
   * Manages the state machine for plan execution, including processing
   * new plans, executing current plans, handling cancellations, and
   * reporting results.
   */
  void execution_cycle();

  // One step of execution_cycle's state machine
  void execution_step();

  // The period of feedback_rate; 5 Hz if it is not positive
  std::chrono::nanoseconds feedback_period();

  /**
   * @brief Updates the remaining plan based on action execution status.
   *
   * @param[in,out] runtime_info Runtime information to update.
   */
  void update_plan(PlanRuntineInfo & runtime_info);

  /**
   * @brief Initializes runtime information for a new plan.
   *
   * @param[in,out] runtime_info Runtime information to initialize.
   * @return True if initialization was successful, false otherwise.
   */
  bool init_plan_for_execution(PlanRuntineInfo & runtime_info);
  bool init_plan_for_execution_impl(PlanRuntineInfo & runtime_info);

  /**
   * @brief Reinitializes runtime information for replanning.
   *
   * @param[in,out] runtime_info Runtime information to reinitialize.
   * @return True if reinitialization was successful, false otherwise.
   */
  bool replan_for_execution(PlanRuntineInfo & runtime_info);
  bool replan_for_execution_impl(PlanRuntineInfo & runtime_info);

  /**
   * @brief Executes the current plan using the behavior tree.
   */
  void execute_plan();

  /**
   * @brief Builds a behavior tree from the current plan.
   *
   * @param[in,out] runtime_info Runtime information containing the plan.
   * @param[in] initial_state The state the plan starts from, if not the current one.
   * @param[in] report_errors Whether a plan that cannot be executed is an error to log.
   * @return True if tree creation was successful, false otherwise.
   */
  bool get_tree_from_plan(
    PlanRuntineInfo & runtime_info,
    const std::optional<std::pair<std::vector<plansys2::Predicate>,
    std::vector<plansys2::Function>>> & initial_state = std::nullopt,
    bool report_errors = true);

  /**
   * @brief Creates runtime information structures for a plan.
   *
   * @param[out] runtime_info Runtime information to create.
   */
  void create_plan_runtime_info(PlanRuntineInfo & runtime_info);

  /**
   * @brief Take out of a new plan the actions the previous one just finished.
   *
   * A plan computed while an action ran can reach the executor after it ended: it starts with
   * that action, whose requirements hold no more. The first time each such action appears is
   * taken out; a later one is the plan doing it again.
   *
   * @param[in,out] runtime_info Runtime information with the new plan.
   * @param[in] previous_action_map The actions of the previous plan.
   * @return The actions taken out, with their new ids and the information of when they ran.
   */
  std::vector<std::pair<std::string, ActionExecutionInfo>> skip_finished_actions(
    PlanRuntineInfo & runtime_info,
    const std::map<std::string, ActionExecutionInfo> & previous_action_map);

  /**
   * @brief Cancels all currently running actions.
   *
   * @param[in,out] runtime_info Runtime information containing the actions.
   */
  void cancel_all_running_actions(PlanRuntineInfo & runtime_info);
  // The continuous effects still active of a plan's actions end, whatever ended the plan
  void stop_continuous_effects(PlanRuntineInfo & runtime_info);
  // PDDL+: what happens depends on time, so actions start at their planned times
  bool is_timed(const PlanRuntineInfo & runtime_info);

  /**
   * @brief Takes back the at start effects of the actions that started and did not end.
   *
   * Does nothing if the parameter restore_at_start_effects is false.
   *
   * @param[in,out] action_map The actions of the plan.
   */
  void restore_at_start_effects(std::map<std::string, ActionExecutionInfo> & action_map);

  /**
   * @brief Takes back the at start effects of one action, if it started and did not end.
   *
   * @param[in] action_id The id of the action, for the log.
   * @param[in,out] info The execution information of the action.
   */
  void restore_at_start_effects(const std::string & action_id, ActionExecutionInfo & info);

  /**
   * @brief Add Groot2 monitor to publish BT status changes
   * @param tree BT to monitor
   * @param server_port Groot2 Server port, first of the pair (server_port, publisher_port)
   */
  void add_groot_monitoring(BT::Tree * tree, uint16_t server_port);

  /**
   * @brief Reset Groot2 monitor
   */
  void reset_groot_monitor();

  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  ActionHub::Ptr action_hub_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr
    execution_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Plan>::SharedPtr executing_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Plan>::SharedPtr remaining_plan_pub_;
  // What it does while a plan runs, and why
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ExecutionEvent>::SharedPtr
    trace_pub_;
  std::shared_ptr<ExecutionTrace> trace_;

  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_plan_action_server_;
  rclcpp::Service<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr dotgraph_pub_;

  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_remaining_plan_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblem>::SharedPtr get_replanning_problem_service_;
  // Made by the execution thread, that calls the experts: a service callback that called them
  // would wait for itself if they share its thread. The requests waiting for it
  std::mutex replanning_mutex_;
  std::vector<std::pair<std::shared_ptr<rclcpp::Service<plansys2_msgs::srv::GetProblem>>,
    std::shared_ptr<rmw_request_id_t>>> replanning_requests_;
  void make_replanning_problem();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr domain_sub_;
  // Each change of the knowledge is a new state for the constraints
  rclcpp::Subscription<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_sub_;
  std::atomic<bool> knowledge_changed_ {true};

  rclcpp::TimerBase::SharedPtr execution_timer_;

  // What execution_step published last, to publish only what changed or is due
  std::optional<plansys2_msgs::msg::Plan> last_executing_plan_;
  std::chrono::steady_clock::time_point next_feedback_time_;

  bool cancel_plan_requested_ {false};
  bool replan_requested_ {false};
  // Flags shared between execution_cycle's thread and the ROS callbacks (#435)
  std::atomic<bool> new_plan_received_ {false};
  bool domain_baseline_seen_ {false};
  // Set by domain_topic_callback, consumed by execution_cycle
  std::atomic<bool> domain_changed_ {false};

  std::atomic<bool> node_running_ {false};

  // The constraints of the plan just set up, from the domain and the problem now
  void start_constraints(PlanRuntineInfo & runtime_info);
  // Whether the constraints hold now: the reason if they do not
  std::optional<std::string> check_constraints(BT::NodeStatus status);
  // The time of the constraints now
  double constraints_time(const PlanRuntineInfo & runtime_info);
  // The continuous effects of a running action stop acting
  void stop_continuous_effect(const std::string & action_id, const ActionExecutionInfo & info);
  std::thread execution_thread_;

  std::list<std::shared_ptr<GoalHandleExecutePlan>> goal_handlers_;
  std::shared_ptr<GoalHandleExecutePlan> current_goal_handle_;
  std::shared_ptr<GoalHandleExecutePlan> new_goal_handle_;
  // Protects new_goal_handle_, set by handle_accepted and taken by execution_cycle
  std::mutex goal_mutex_;

  static const int STATE_IDLE = 0;
  static const int STATE_EXECUTING = 1;
  static const int STATE_REPLANNING = 2;
  static const int STATE_ABORTING = 3;
  static const int STATE_CANCELLED = 4;
  static const int STATE_FAILED = 5;
  static const int STATE_SUCCEDED = 6;
  static const int STATE_ERROR = 7;
  std::atomic<int> executor_state_;

  // Owned by execution_cycle's thread
  PlanRuntineInfo runtime_info_;

  // Copy of the plan for the services. They run in the ROS executor, which in the
  // monolithic bringup also serves the requests execution_cycle makes while ticking,
  // so they must never wait for the execution thread: this lock is only held briefly
  std::mutex snapshot_mutex_;
  // The last goal accepted, until it runs or ends: get_plan answers it while a long plan is
  // set up, or a client would take it as lost
  std::shared_ptr<GoalHandleExecutePlan> starting_goal_;
  plansys2_msgs::msg::Plan starting_plan_;
  void starting_done(const std::shared_ptr<GoalHandleExecutePlan> & goal);
  plansys2_msgs::msg::Plan complete_plan_snapshot_;
  plansys2_msgs::msg::Plan remaining_plan_snapshot_;
  std::vector<plansys2_msgs::msg::Tree> ordered_sub_goals_snapshot_;
  // What the at start effects of the running actions changed
  std::vector<FactChange> running_changes_snapshot_;

  void update_snapshot();

  // Takes the goal waiting to start, if any, clearing new_plan_received_ with it
  std::shared_ptr<GoalHandleExecutePlan> take_new_goal();
  // Ends a goal replaced by another one
  static void finish_replaced_goal(
    const std::shared_ptr<GoalHandleExecutePlan> & goal,
    const std::shared_ptr<ExecutePlan::Result> & result);
  // The action executors and the BT blackboard live in this node: an owning pointer
  // from them would keep the node alive forever (#430)
  rclcpp_lifecycle::LifecycleNode::SharedPtr non_owning_this()
  {
    return rclcpp_lifecycle::LifecycleNode::SharedPtr(
      rclcpp_lifecycle::LifecycleNode::SharedPtr(), this);
  }
  void start_execution_thread();
  // Stops execution_cycle and fails the goals it was handling
  void stop_execution_thread();
  // The waits of the clients of the node end at once while interrupted
  void set_clients_interrupted(bool interrupted);
  // Aborts the running and the waiting goal, if any, and returns to idle
  void fail_current_goals();

  // Groot2 monitor
  std::unique_ptr<BT::Groot2Publisher> groot_monitor_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
