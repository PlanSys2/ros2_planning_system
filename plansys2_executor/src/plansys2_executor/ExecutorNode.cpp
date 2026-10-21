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

#include <filesystem>

#include <algorithm>
#include <cstdio>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <vector>
#include <utility>
#include <optional>
#include <cmath>

#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_core/PackagePaths.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/check_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_timeout_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/restore_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"
#include "plansys2_executor/BTUtils.hpp"
#include "plansys2_executor/JSONUtils.hpp"

namespace plansys2
{

using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using namespace std::chrono_literals;

ExecutorNode::ExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("executor", options),
  bt_builder_loader_("plansys2_executor", "plansys2::BTBuilder"),
  executor_state_(STATE_IDLE)
{
  using namespace std::placeholders;

  this->declare_parameter<std::string>("default_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("default_start_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("default_end_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("bt_builder_plugin", "");
  this->declare_parameter<int>("action_time_precision", 3);
  // Seconds a plan waits for a component that is not active before it fails (0: for ever)
  this->declare_parameter<double>("component_timeout", 30.0);
  // STNBTBuilder: true starts each action as soon as its dependencies happened, as durations
  // are uncertain in robotics; false keeps the planned times as minimums
  this->declare_parameter<bool>("stn_relaxed_timing", true);
  this->declare_parameter<bool>("enable_dotgraph_legend", true);
  this->declare_parameter<bool>("print_graph", false);
  // Feedback, execution info, remaining plan and dot graph, in Hz; the tree ticks at 50 Hz
  this->declare_parameter<double>("feedback_rate", 5.0);
  // A plan that fails, is cancelled or is replaced takes back the at start effects of the
  // actions that started and did not end
  this->declare_parameter<bool>("restore_at_start_effects", true);
  // An action whose at start requirements do not hold this long fails, instead of waiting for
  // ever (0: no limit)
  this->declare_parameter<double>("at_start_requirements_timeout", 30.0);
  // PDDL+ plans: actions at their planned times (always if the domain has processes, events or
  // continuous effects), and how long the goal may take to hold once the plan has ended
  this->declare_parameter<bool>("timed_execution", false);
  // Each event of the execution, and why, also in the log
  this->declare_parameter<bool>("log_execution_trace", true);
  this->declare_parameter<double>("plan_end_goal_timeout", 5.0);
  this->declare_parameter("action_timeouts.actions", std::vector<std::string>{});
  // Declaring individual action parameters so they can be queried on the command line
  auto action_timeouts_actions = this->get_parameter("action_timeouts.actions").as_string_array();
  for (auto action : action_timeouts_actions) {
    this->declare_parameter<double>(
      "action_timeouts." + action + ".duration_overrun_percentage",
      0.0);
  }

  this->declare_parameter<bool>("enable_groot_monitoring", false);
  this->declare_parameter<int>("server_port", 1800);

  // An ended goal keeps its result a minute: a client whose result was lost asks for it again
  auto server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(60);
  execute_plan_action_server_ = rclcpp_action::create_server<ExecutePlan>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "execute_plan",
    std::bind(&ExecutorNode::handle_goal, this, _1, _2),
    std::bind(&ExecutorNode::handle_cancel, this, _1),
    std::bind(&ExecutorNode::handle_accepted, this, _1),
    server_options);

  get_ordered_sub_goals_service_ = create_service<plansys2_msgs::srv::GetOrderedSubGoals>(
    "executor/get_ordered_sub_goals",
    guarded<plansys2_msgs::srv::GetOrderedSubGoals>(
      get_logger(),
      std::bind(
        &ExecutorNode::get_ordered_sub_goals_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
    "executor/get_plan",
    guarded<plansys2_msgs::srv::GetPlan>(
      get_logger(),
      std::bind(
        &ExecutorNode::get_plan_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_remaining_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
    "executor/get_remaining_plan",
    guarded<plansys2_msgs::srv::GetPlan>(
      get_logger(),
      std::bind(
        &ExecutorNode::get_remaining_plan_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3)));

  get_replanning_problem_service_ = create_service<plansys2_msgs::srv::GetProblem>(
    "executor/get_replanning_problem",
    guarded_deferred<plansys2_msgs::srv::GetProblem>(
      get_logger(),
      [this](
        std::shared_ptr<rclcpp::Service<plansys2_msgs::srv::GetProblem>> service,
        std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<plansys2_msgs::srv::GetProblem::Request>) {
        get_replanning_problem_service_callback(service, header);
    }));
}

ExecutorNode::~ExecutorNode()
{
  // execution_cycle uses this object: it must be gone before anything is destroyed
  stop_execution_thread();
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ExecutorNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  auto default_action_bt_xml_filename =
    this->get_parameter("default_action_bt_xml_filename").as_string();
  if (default_action_bt_xml_filename.empty()) {
    auto pkg_path =
      plansys2::share_path("plansys2_executor");
    default_action_bt_xml_filename =
      (pkg_path / "behavior_trees" / "plansys2_action_bt.xml").string();
  }

  std::ifstream action_bt_ifs(default_action_bt_xml_filename);
  if (!action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << default_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  action_bt_xml_.assign(
    std::istreambuf_iterator<char>(action_bt_ifs), std::istreambuf_iterator<char>());

  auto default_start_action_bt_xml_filename =
    this->get_parameter("default_start_action_bt_xml_filename").as_string();
  if (default_start_action_bt_xml_filename.empty()) {
    auto pkg_path =
      plansys2::share_path("plansys2_executor");
    default_start_action_bt_xml_filename =
      (pkg_path / "behavior_trees" / "plansys2_start_action_bt.xml").string();
  }

  std::ifstream start_action_bt_ifs(default_start_action_bt_xml_filename);
  if (!start_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error openning [" << default_start_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  start_action_bt_xml_.assign(
    std::istreambuf_iterator<char>(start_action_bt_ifs), std::istreambuf_iterator<char>());

  auto default_end_action_bt_xml_filename =
    this->get_parameter("default_end_action_bt_xml_filename").as_string();
  if (default_end_action_bt_xml_filename.empty()) {
    auto pkg_path =
      plansys2::share_path("plansys2_executor");
    default_end_action_bt_xml_filename =
      (pkg_path / "behavior_trees" / "plansys2_end_action_bt.xml").string();
  }

  std::ifstream end_action_bt_ifs(default_end_action_bt_xml_filename);
  if (!end_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error openning [" << default_end_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  end_action_bt_xml_.assign(
    std::istreambuf_iterator<char>(end_action_bt_ifs), std::istreambuf_iterator<char>());

  dotgraph_pub_ = this->create_publisher<std_msgs::msg::String>("dot_graph", 1);
  execution_info_pub_ = create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(
    "action_execution_info", 100);
  executing_plan_pub_ = create_publisher<plansys2_msgs::msg::Plan>(
    "executing_plan", rclcpp::QoS(100).transient_local());
  remaining_plan_pub_ = create_publisher<plansys2_msgs::msg::Plan>(
    "remaining_plan", rclcpp::QoS(100));
  // A viewer that comes later still sees the last events
  trace_pub_ = create_publisher<plansys2_msgs::msg::ExecutionEvent>(
    "executor/execution_trace", rclcpp::QoS(1000).transient_local());
  trace_ = std::make_shared<ExecutionTrace>(
    non_owning_this(), trace_pub_, get_parameter("log_execution_trace").as_bool());

  // On this node: the executor is one node, whatever it uses inside
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(*this);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(*this);
  planner_client_ = std::make_shared<plansys2::PlannerClient>(*this);
  // A plan does not wait for ever for a component that is gone: it fails after this
  const auto max_wait = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 * get_parameter("component_timeout").as_double()));
  domain_client_->set_max_wait(max_wait);
  problem_client_->set_max_wait(max_wait);
  planner_client_->set_max_wait(max_wait);

  // One actions_hub publisher and subscription for all the actions, kept between plans
  action_hub_ = ActionHub::get(non_owning_this());

  // A new subscription gets the latched domain again: that is not a change
  domain_baseline_seen_ = false;
  domain_sub_ = create_subscription<std_msgs::msg::String>(
    "domain_expert/domain",
    rclcpp::QoS(100).transient_local(),
    std::bind(&ExecutorNode::domain_topic_callback, this, std::placeholders::_1));
  knowledge_sub_ = create_subscription<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge", rclcpp::QoS(100).transient_local(),
    [this](plansys2_msgs::msg::Knowledge::SharedPtr) {knowledge_changed_ = true;});

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  dotgraph_pub_->on_activate();
  execution_info_pub_->on_activate();
  executing_plan_pub_->on_activate();
  remaining_plan_pub_->on_activate();
  trace_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  start_execution_thread();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  stop_execution_thread();
  dotgraph_pub_->on_deactivate();
  executing_plan_pub_->on_deactivate();
  remaining_plan_pub_->on_deactivate();
  trace_pub_->on_deactivate();
  reset_groot_monitor();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  stop_execution_thread();
  dotgraph_pub_.reset();
  executing_plan_pub_.reset();
  remaining_plan_pub_.reset();
  action_hub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  stop_execution_thread();
  dotgraph_pub_.reset();
  executing_plan_pub_.reset();
  remaining_plan_pub_.reset();
  action_hub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
ExecutorNode::get_ordered_sub_goals_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response)
{
  (void)request;
  (void)request_header;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  response->sub_goals = ordered_sub_goals_snapshot_;
  response->success = true;
}

void
ExecutorNode::get_ordered_subgoals(PlanRuntineInfo & runtime_info)
{
  auto goal = problem_client_->getGoal();
  auto local_predicates = problem_client_->getPredicates();
  auto local_functions = problem_client_->getFunctions();

  std::vector<uint32_t> unordered_subgoals = parser::pddl::getSubtreeIds(goal);

  // just in case some goals are already satisfied
  for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end(); ) {
    if (check(goal, local_predicates, local_functions, *it)) {
      plansys2_msgs::msg::Tree new_goal;
      parser::pddl::fromString(new_goal, "(and " + parser::pddl::toString(goal, (*it)) + ")");
      runtime_info.ordered_sub_goals.push_back(new_goal);
      it = unordered_subgoals.erase(it);
    } else {
      ++it;
    }
  }

  // Once for the whole plan: it was asked for every action
  const auto actions = domain_client_->getActions();
  for (const auto & plan_item : runtime_info.complete_plan.items) {
    const auto action = BTBuilder::plan_item_action(*domain_client_, actions, plan_item);
    if (action.is_empty()) {
      continue;
    }
    plansys2::apply(action.get_at_start_effects(), local_predicates, local_functions);
    plansys2::apply(action.get_at_end_effects(), local_predicates, local_functions);


    for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end(); ) {
      if (check(goal, local_predicates, local_functions, *it)) {
        plansys2_msgs::msg::Tree new_goal;
        parser::pddl::fromString(new_goal, "(and " + parser::pddl::toString(goal, (*it)) + ")");
        runtime_info.ordered_sub_goals.push_back(new_goal);
        it = unordered_subgoals.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void
ExecutorNode::get_plan_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response)
{
  (void)request_header;
  (void)request;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (starting_goal_) {
    response->success = true;
    response->plan = starting_plan_;
  } else if (executor_state_ != STATE_IDLE) {
    // Running, or ending (failed, cancelled...): the plan is the one of the goal until it ends
    response->success = true;
    response->plan = complete_plan_snapshot_;
  } else {
    response->success = false;
  }
}

void
ExecutorNode::get_remaining_plan_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response)
{
  (void)request;
  (void)request_header;
  if (executor_state_ == STATE_EXECUTING) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    response->success = true;
    response->plan = remaining_plan_snapshot_;
  } else {
    response->success = false;
    response->error_info = "Not executing plan";
  }
}

void
ExecutorNode::create_plan_runtime_info(PlanRuntineInfo & runtime_info)
{
  runtime_info.action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();
  auto action_timeout_actions = this->get_parameter("action_timeouts.actions").as_string_array();

  (*runtime_info.action_map)[":0"] = ActionExecutionInfo();
  (*runtime_info.action_map)[":0"].action_executor = ActionExecutor::make_shared("(INIT)",
    non_owning_this());
  (*runtime_info.action_map)[":0"].action_executor->set_internal_status(
    ActionExecutor::Status::SUCCESS);
  (*runtime_info.action_map)[":0"].at_start_effects_applied = true;
  (*runtime_info.action_map)[":0"].at_end_effects_applied = true;
  (*runtime_info.action_map)[":0"].at_start_effects_applied_time = now();
  (*runtime_info.action_map)[":0"].at_end_effects_applied_time = now();

  // Once for the whole plan: it was asked for every action
  const auto actions = domain_client_->getActions();
  for (const auto & plan_item : runtime_info.complete_plan.items) {
    // The parsing below assumes "(name args...)"; anything else must not reach it
    const auto & action = plan_item.action;
    if (action.size() < 3 || action.front() != '(' || action.back() != ')' ||
      action.find_first_not_of(" \t()") == std::string::npos)
    {
      throw std::runtime_error("malformed action in plan: [" + action + "]");
    }

    auto index = BTBuilder::to_action_id(plan_item, action_time_precision());
    (*runtime_info.action_map)[index] = ActionExecutionInfo();
    (*runtime_info.action_map)[index].plan_item = plan_item;
    (*runtime_info.action_map)[index].action_executor =
      ActionExecutor::make_shared(plan_item.action, non_owning_this());

    (*runtime_info.action_map)[index].action_info =
      BTBuilder::plan_item_action(*domain_client_, actions, plan_item);
    if ((*runtime_info.action_map)[index].action_info.is_empty()) {
      throw std::runtime_error("action not in the domain: " + plan_item.action);
    }

    std::string action_name = (*runtime_info.action_map)[index].action_info.get_action_name();
    (*runtime_info.action_map)[index].duration = plan_item.duration;
    (*runtime_info.action_map)[index].action_executor->set_planned_duration(plan_item.duration);

    if (std::find(
        action_timeout_actions.begin(), action_timeout_actions.end(),
        action_name) != action_timeout_actions.end() &&
      this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage"))
    {
      (*runtime_info.action_map)[index].duration_overrun_percentage = this->get_parameter(
        "action_timeouts." + action_name + ".duration_overrun_percentage").as_double();
    }
    RCLCPP_DEBUG(
      get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
      (*runtime_info.action_map)[index].duration_overrun_percentage);
  }

  runtime_info.ordered_sub_goals = {};
  get_ordered_subgoals(runtime_info);
}

bool
ExecutorNode::get_tree_from_plan(
  PlanRuntineInfo & runtime_info,
  const std::optional<std::pair<std::vector<plansys2::Predicate>,
  std::vector<plansys2::Function>>> & initial_state,
  bool report_errors)
{
  auto bt_builder_plugin = this->get_parameter("bt_builder_plugin").as_string();
  if (bt_builder_plugin.empty()) {
    bt_builder_plugin = "STNBTBuilder";
  }

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  try {
    bt_builder = bt_builder_loader_.createSharedInstance("plansys2::" + bt_builder_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());
    return false;
  }
  // Our clients: otherwise each plan would create, and later destroy, two more nodes
  bt_builder->set_clients(domain_client_, problem_client_);
  if (initial_state) {
    bt_builder->set_initial_state(initial_state->first, initial_state->second);
  }

  if (bt_builder_plugin == "STNBTBuilder") {
    // Start and end of each action are separate BT units
    bt_builder->initialize(start_action_bt_xml_, end_action_bt_xml_, action_time_precision());
  } else {
    // Every builder needs the action BT template (#431), and the precision of the ids
    bt_builder->initialize(action_bt_xml_, "", action_time_precision());
  }

  // The timed facts of the problem now, in the time of the plan: it was read before
  const auto now_time = now();
  const rclcpp::Time plan_stamp(runtime_info.complete_plan.stamp, now_time.get_clock_type());
  if (plan_stamp.nanoseconds() > 0 && plan_stamp <= now_time) {
    bt_builder->set_time_offset((now_time - plan_stamp).seconds());
  }

  auto bt_xml_tree = bt_builder->get_tree(runtime_info.complete_plan);
  if (bt_xml_tree.empty()) {
    if (report_errors) {
      RCLCPP_ERROR(get_logger(), "Error computing behavior tree!");
    }
    return false;
  }

  auto action_graph = bt_builder->get_graph();
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
    runtime_info.action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
    this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  // The last tree, to look at: in the temporary directory, one per namespace
  {
    std::error_code error;
    const auto dir = std::filesystem::temp_directory_path(error) / "plansys2" /
      std::filesystem::path(get_namespace()).relative_path();
    std::filesystem::create_directories(dir, error);
    std::ofstream out(dir / "bt.xml");
    out << bt_xml_tree;
  }

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteAction>("ExecuteAction");
  factory.registerNodeType<WaitAction>("WaitAction");
  factory.registerNodeType<CheckAction>("CheckAction");
  factory.registerNodeType<CheckOverAllReq>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReq>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReq>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffect>("ApplyAtStartEffect");
  factory.registerNodeType<RestoreAtStartEffect>("RestoreAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffect>("ApplyAtEndEffect");
  factory.registerNodeType<CheckTimeout>("CheckTimeout");

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", runtime_info.action_map);
  blackboard->set("action_graph", action_graph);
  blackboard->set("node", non_owning_this());
  blackboard->set("domain_client", domain_client_);
  blackboard->set("problem_client", problem_client_);
  blackboard->set("bt_builder", bt_builder);
  blackboard->set("stn_relaxed_timing", this->get_parameter("stn_relaxed_timing").as_bool());
  // Timed, the plan counts from when its problem was read: its timed facts and what changes by
  // itself do; what was due before it starts runs at once
  const auto execution_start = now();
  runtime_info.plan_start = execution_start;
  const rclcpp::Time stamp(runtime_info.complete_plan.stamp, execution_start.get_clock_type());
  double read_before = 0.0;
  if (is_timed(runtime_info) && stamp.nanoseconds() > 0 && stamp <= execution_start) {
    runtime_info.plan_start = stamp;
    read_before = (execution_start - stamp).seconds();
  }
  runtime_info.tree_done = false;
  blackboard->set("plan_start", runtime_info.plan_start);
  blackboard->set("execution_start", execution_start);
  blackboard->set("timed_execution", is_timed(runtime_info));
  blackboard->set("trace", trace_);
  trace_->set_plan_start(runtime_info.plan_start);
  auto builder_name = get_parameter("bt_builder_plugin").as_string();
  char read[64] = "";
  if (read_before > 0.0) {
    std::snprintf(read, sizeof(read), "its problem read %.3f s before, ", read_before);
  }
  trace_->event(
    ExecutionTrace::Event::PLAN_START, "",
    std::string(is_timed(runtime_info) ? "timed, each action at its time" :
    "each action as soon as it can") + ", " + read +
    std::to_string(runtime_info.complete_plan.items.size()) + " actions, " +
    (builder_name.empty() ? std::string("STNBTBuilder") : builder_name));
  // Added blackboard keys for compatibility with other nodes
  blackboard->set("bt_loop_duration", std::chrono::milliseconds(200));
  blackboard->set("server_timeout", std::chrono::milliseconds(250));
  blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(1000));

  // If a new tree is created, than the Groot2 Publisher must be destroyed
  reset_groot_monitor();

  runtime_info.current_tree = std::make_shared<TreeInfo>();
  *runtime_info.current_tree = {
    factory.createTreeFromText(bt_xml_tree, blackboard), blackboard, bt_builder};

  bool enable_groot_monitoring = get_parameter("enable_groot_monitoring").as_bool();
  int server_port = get_parameter("server_port").as_int();
  if (enable_groot_monitoring) {
    RCLCPP_INFO(get_logger(), "Enabling Groot2 monitoring on port: %d", server_port);
    add_groot_monitoring(&runtime_info.current_tree->tree, server_port);
  }

  return runtime_info.current_tree != nullptr;
}

bool
ExecutorNode::is_timed(const PlanRuntineInfo & runtime_info)
{
  if (get_parameter("timed_execution").as_bool() || runtime_info.complete_plan.has_end) {
    return true;
  }
  // PDDL3: (within 10 ...) is 10 s of the plan
  if (runtime_info.constraints && runtime_info.constraints->timed()) {
    return true;
  }
  if (!domain_client_->getProcesses().empty() || !domain_client_->getEvents().empty()) {
    return true;
  }
  if (runtime_info.action_map) {
    for (const auto & [id, info] : *runtime_info.action_map) {
      (void)id;
      if (!info.action_info.is_empty() &&
        !info.action_info.get_continuous_effects().nodes.empty())
      {
        return true;
      }
    }
  }
  return false;
}

void
ExecutorNode::stop_continuous_effects(PlanRuntineInfo & runtime_info)
{
  if (!runtime_info.action_map) {
    return;
  }
  for (const auto & [id, info] : *runtime_info.action_map) {
    stop_continuous_effect(id, info);
  }
}

void
ExecutorNode::stop_continuous_effect(
  const std::string & action_id,
  const ActionExecutionInfo & info)
{
  if (!info.action_info.is_empty() &&
    !info.action_info.get_continuous_effects().nodes.empty() &&
    info.at_start_effects_applied && !info.at_end_effects_applied)
  {
    problem_client_->setContinuousEffect(action_id, plansys2_msgs::msg::Tree(), false);
  }
}

void
ExecutorNode::start_constraints(PlanRuntineInfo & runtime_info)
{
  runtime_info.constraints = std::make_shared<ConstraintMonitor>(
    domain_client_->getDomain(), problem_client_->getProblem());
  if (runtime_info.constraints->empty()) {
    runtime_info.constraints.reset();
    return;
  }
  for (const auto & constraint : runtime_info.constraints->describe()) {
    RCLCPP_INFO(get_logger(), "[%s] Following the constraint %s", get_name(), constraint.c_str());
  }
  knowledge_changed_ = true;  // The state it starts from
}

double
ExecutorNode::constraints_time(const PlanRuntineInfo & runtime_info)
{
  // From the start of the plan, shifted by the actions that started late
  rclcpp::Time start = runtime_info.plan_start;
  if (runtime_info.current_tree) {
    std::ignore = runtime_info.current_tree->tree.rootBlackboard()->get("plan_start", start);
  }
  return (now() - start).seconds() + runtime_info.constraints_offset;
}

std::optional<std::string>
ExecutorNode::check_constraints(BT::NodeStatus status)
{
  if (!runtime_info_.constraints) {
    return {};
  }
  const double t = constraints_time(runtime_info_);

  auto holds = [this](const plansys2_msgs::msg::Tree & condition) {
      return check_in_expert(condition, problem_client_);
    };
  // At the end, the last state as it is, whether its message came or not
  const bool done = status == BT::NodeStatus::SUCCESS;
  auto violated = (knowledge_changed_.exchange(false) || done) ?
    runtime_info_.constraints->update(t, holds) : runtime_info_.constraints->tick(t);
  if (!violated && done) {
    violated = runtime_info_.constraints->finish(t);
  }
  return violated;
}

bool
ExecutorNode::init_plan_for_execution(PlanRuntineInfo & runtime_info)
{
  // Anything wrong with the plan (unknown actions, bad BT...) fails the goal (#431)
  try {
    return init_plan_for_execution_impl(runtime_info);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Cannot set up the plan: %s", e.what());
    return false;
  }
}

bool
ExecutorNode::init_plan_for_execution_impl(PlanRuntineInfo & runtime_info)
{
  cancel_plan_requested_ = false;
  replan_requested_ = false;

  if (runtime_info.action_map != nullptr) {
    for (auto & entry : *runtime_info.action_map) {
      ActionExecutionInfo & action_info = entry.second;
      if (!action_info.action_executor) {
        continue;
      }
      action_info.action_executor->cancel();
      action_info.action_executor->clean_up();
      action_info.action_executor = nullptr;
    }
    runtime_info.action_map->clear();
  }

  create_plan_runtime_info(runtime_info);

  start_constraints(runtime_info);
  bool plan_success = get_tree_from_plan(runtime_info);

  if (!plan_success) {
    return false;
  }

  return true;
}

bool
ExecutorNode::replan_for_execution(PlanRuntineInfo & runtime_info)
{
  // Anything wrong with the plan (unknown actions, bad BT...) fails the goal (#431)
  try {
    return replan_for_execution_impl(runtime_info);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Cannot set up the plan: %s", e.what());
    return false;
  }
}

namespace
{
// The facts as they were before these changes, as undo does on the problem expert
void undo_in_state(
  const std::vector<FactChange> & changes, std::vector<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions)
{
  for (auto change = changes.rbegin(); change != changes.rend(); ++change) {
    if (change->fact.node_type == plansys2_msgs::msg::Node::PREDICATE) {
      auto it = std::find_if(
        predicates.begin(), predicates.end(), [&](const plansys2::Predicate & p) {
          return parser::pddl::checkNodeEquality(p, change->fact);
        });
      if (change->existed_before && it == predicates.end()) {
        plansys2::Predicate fact(change->fact);
        fact.negate = false;
        predicates.push_back(fact);
      } else if (!change->existed_before && it != predicates.end()) {
        predicates.erase(it);
      }
      continue;
    }
    auto it = std::find_if(
      functions.begin(), functions.end(), [&](const plansys2::Function & f) {
        return parser::pddl::checkNodeEquality(f, change->fact);
      });
    if (it == functions.end()) {
      continue;
    }
    if (change->modifier_type == plansys2_msgs::msg::Node::INCREASE ||
      change->modifier_type == plansys2_msgs::msg::Node::DECREASE)
    {
      it->value -= change->value_after - change->value_before;
    } else if (std::fabs(it->value - change->value_after) < 1e-9) {
      it->value = change->value_before;
    }
  }
}

std::string action_expression(const std::string & action_id)
{
  return action_id.substr(0, action_id.find(':'));
}
}  // namespace

void
ExecutorNode::get_replanning_problem_service_callback(
  std::shared_ptr<rclcpp::Service<plansys2_msgs::srv::GetProblem>> service,
  std::shared_ptr<rmw_request_id_t> request_header)
{
  std::lock_guard<std::mutex> lock(replanning_mutex_);
  replanning_requests_.push_back({service, request_header});
}

void
ExecutorNode::make_replanning_problem()
{
  decltype(replanning_requests_) requests;
  {
    std::lock_guard<std::mutex> lock(replanning_mutex_);
    requests.swap(replanning_requests_);
  }
  if (requests.empty()) {
    return;
  }
  std::vector<FactChange> changes;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    changes = running_changes_snapshot_;
  }
  plansys2_msgs::srv::GetProblem::Response response;
  response.stamp = now();
  try {
    // The problem of now, as a problem expert of its own, before what the running actions began
    auto domain = std::make_shared<plansys2::DomainExpert>(domain_client_->getDomain());
    plansys2::ProblemExpert problem(domain);
    if (!problem.addProblem(problem_client_->getProblem())) {
      response.success = false;
      response.error_info = "the problem of the problem expert cannot be read";
    } else {
      for (auto change = changes.rbegin(); change != changes.rend(); ++change) {
        if (change->fact.node_type == plansys2_msgs::msg::Node::PREDICATE) {
          plansys2::Predicate fact(change->fact);
          fact.negate = false;
          if (change->existed_before) {
            problem.addPredicate(fact);
          } else {
            problem.removePredicate(fact);
          }
          continue;
        }
        auto function = problem.getFunction(parser::pddl::toString(change->fact));
        if (!function) {
          continue;
        }
        const bool relative = change->modifier_type == plansys2_msgs::msg::Node::INCREASE ||
          change->modifier_type == plansys2_msgs::msg::Node::DECREASE;
        if (!change->object_before.empty() || !change->object_after.empty()) {
          function->object = change->object_before;
        } else if (relative) {
          function->value -= change->value_after - change->value_before;
        } else if (std::fabs(function->value - change->value_after) < 1e-9) {
          function->value = change->value_before;
        }
        problem.updateFunction(function.value());
      }
      response.problem = problem.getProblem();
      response.success = true;
    }
  } catch (const std::exception & e) {
    response.success = false;
    response.error_info = e.what();
  }
  for (auto & [service, header] : requests) {
    service->send_response(*header, response);
  }
}

bool
ExecutorNode::replan_for_execution_impl(PlanRuntineInfo & runtime_info)
{
  cancel_plan_requested_ = false;
  replan_requested_ = false;

  // Actions that do not go on in the new plan are taken back first: the new plan is checked
  // against the state, and it starts from where they began
  std::set<std::string> new_ids;
  for (const auto & item : runtime_info.remaining_plan.items) {
    new_ids.insert(BTBuilder::to_action_id(item, action_time_precision()));
  }
  std::string going_on, dropped;
  for (auto & [action_id, info] : *runtime_info.action_map) {
    const bool running = info.action_executor &&
      info.action_executor->get_internal_status() == ActionExecutor::RUNNING;
    const bool goes_on = running &&
      new_ids.count(action_id.substr(0, action_id.find(':')) + ":0") > 0;
    if (running) {
      (goes_on ? going_on : dropped) += " " + action_id;
    }
    if (!goes_on) {
      stop_continuous_effect(action_id, info);
      restore_at_start_effects(action_id, info);
    }
  }
  trace_->event(
    ExecutionTrace::Event::REPLAN, "",
    "going on:" + (going_on.empty() ? std::string(" none") : going_on) + "; dropped:" +
    (dropped.empty() ? std::string(" none") : dropped));

  std::map<std::string, ActionExecutionInfo> previous_action_map = *runtime_info.action_map;

  // The actions that go on started before the new plan: it starts from the state before their
  // at start effects, or it would not hold their at start requirements
  std::optional<std::pair<std::vector<plansys2::Predicate>, std::vector<plansys2::Function>>>
  initial_state;
  for (auto & [action_id, info] : *runtime_info.action_map) {
    const bool goes_on = info.action_executor &&
      info.action_executor->get_internal_status() == ActionExecutor::RUNNING &&
      new_ids.count(action_expression(action_id) + ":0") > 0;
    if (goes_on && info.at_start_effects_applied && !info.at_start_changes.empty()) {
      if (!initial_state) {
        initial_state.emplace(problem_client_->getPredicates(), problem_client_->getFunctions());
      }
      undo_in_state(info.at_start_changes, initial_state->first, initial_state->second);
    }
  }

  // The actions that go on are taken out of the old tree: when it is destroyed, it halts its
  // nodes, and that would cancel them
  for (auto & [action_id, info] : *runtime_info.action_map) {
    if (info.action_executor &&
      info.action_executor->get_internal_status() == ActionExecutor::RUNNING &&
      new_ids.count(action_id.substr(0, action_id.find(':')) + ":0") > 0)
    {
      info.action_executor = nullptr;
    }
  }

  // Actions done in the previous plan that the new one, computed while they ran, still has
  std::vector<std::pair<std::string, ActionExecutionInfo>> done_again;
  bool plan_success = false;
  // The same constraints go on over the same trajectory: with what they have seen, in its time
  auto previous_constraints = runtime_info.constraints;
  const double constraints_now = previous_constraints ? constraints_time(runtime_info) : 0.0;
  try {
    start_constraints(runtime_info);
    if (previous_constraints && runtime_info.constraints &&
      previous_constraints->describe() == runtime_info.constraints->describe())
    {
      runtime_info.constraints = previous_constraints;
      runtime_info.constraints_offset = constraints_now;
    } else {
      runtime_info.constraints_offset = 0.0;
    }
    create_plan_runtime_info(runtime_info);
    plan_success = get_tree_from_plan(runtime_info, initial_state, false);
    if (!plan_success) {
      done_again = skip_finished_actions(runtime_info, previous_action_map);
      if (!done_again.empty()) {
        create_plan_runtime_info(runtime_info);
      }
      plan_success = get_tree_from_plan(runtime_info, initial_state);
    }
  } catch (...) {
    // The new plan is unusable: actions of the previous one must not keep running
    for (auto & entry : previous_action_map) {
      if (entry.second.action_executor) {
        entry.second.action_executor->cancel();
      }
    }
    throw;
  }

  auto it = previous_action_map.begin();
  while (it != previous_action_map.end()) {
    if (!it->second.action_executor) {
      it = previous_action_map.erase(it);
    } else if (it->second.action_executor->get_internal_status() != ActionExecutor::RUNNING) {
      ActionExecutionInfo & action_info = it->second;
      action_info.action_executor->clean_up();
      action_info.action_executor = nullptr;
      it = previous_action_map.erase(it);
    } else {
      ++it;
    }
  }

  for (auto action_info : previous_action_map) {
    size_t pos = action_info.first.find(':');
    std::string query_action_name = action_info.first.substr(0, pos) + ":0";

    auto match_it = runtime_info.action_map->find(query_action_name);
    if (match_it != runtime_info.action_map->end()) {
      // Its continuous effects act under its id in the new plan, for the time that plan gives it
      const bool acting = !action_info.second.action_info.is_empty() &&
        !action_info.second.action_info.get_continuous_effects().nodes.empty() &&
        action_info.second.at_start_effects_applied && !action_info.second.at_end_effects_applied;
      if (acting && action_info.first != match_it->first) {
        problem_client_->setContinuousEffect(action_info.first, plansys2_msgs::msg::Tree(), false);
        problem_client_->setContinuousEffect(
          match_it->first, match_it->second.action_info.get_continuous_effects(), true);
      }
      const bool timed = is_timed(runtime_info);
      const double new_duration = match_it->second.duration;
      match_it->second.action_executor = action_info.second.action_executor;
      match_it->second.at_start_effects_applied = action_info.second.at_start_effects_applied;
      match_it->second.at_end_effects_applied = action_info.second.at_end_effects_applied;
      match_it->second.at_start_effects_applied_time =
        action_info.second.at_start_effects_applied_time;
      match_it->second.at_start_changes = action_info.second.at_start_changes;
      match_it->second.execution_error_info = action_info.second.execution_error_info;
      match_it->second.duration = action_info.second.duration;
      match_it->second.duration_overrun_percentage =
        action_info.second.duration_overrun_percentage;
      // Timed, it lasts what the new plan gives it from now: its performer is told
      if (timed && action_info.second.at_start_effects_applied) {
        const double total =
          (now() - action_info.second.at_start_effects_applied_time).seconds() + new_duration;
        match_it->second.duration = total;
        match_it->second.action_executor->update_planned_duration(total);
      }
    } else {
      action_info.second.action_executor->cancel();
      action_info.second.action_executor->clean_up();
      action_info.second.action_executor = nullptr;
    }
  }

  // They count as done in this plan too
  for (auto & [action_id, info] : done_again) {
    (*runtime_info.action_map)[action_id] = info;
  }

  if (!plan_success) {
    return false;
  }

  return true;
}

std::vector<std::pair<std::string, ActionExecutionInfo>>
ExecutorNode::skip_finished_actions(
  PlanRuntineInfo & runtime_info,
  const std::map<std::string, ActionExecutionInfo> & previous_action_map)
{
  // The first time each one appears: a later one is the plan doing it again
  std::vector<std::pair<std::string, ActionExecutionInfo>> skipped;
  std::vector<std::string> done;
  for (const auto & [action_id, info] : previous_action_map) {
    if (action_id != ":0" && info.action_executor && info.at_end_effects_applied &&
      info.action_executor->get_internal_status() == ActionExecutor::SUCCESS)
    {
      done.push_back(action_id);
    }
  }
  if (done.empty()) {
    return skipped;
  }

  auto & items = runtime_info.complete_plan.items;
  std::stable_sort(
    items.begin(), items.end(), [](const auto & a, const auto & b) {return a.time < b.time;});
  for (const auto & previous_id : done) {
    auto it = std::find_if(
      items.begin(), items.end(), [&](const plansys2_msgs::msg::PlanItem & item) {
        return item.action == action_expression(previous_id);
      });
    if (it == items.end()) {
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "%s is already done: the new plan was computed while it ran",
      it->action.c_str());
    skipped.emplace_back(
      BTBuilder::to_action_id(*it, action_time_precision()), previous_action_map.at(previous_id));
    items.erase(it);
  }
  runtime_info.remaining_plan = runtime_info.complete_plan;
  return skipped;
}

void
ExecutorNode::cancel_all_running_actions(PlanRuntineInfo & runtime_info)
{
  if (runtime_info.action_map != nullptr) {
    for (auto & entry : *runtime_info.action_map) {
      ActionExecutionInfo & action_info = entry.second;
      // A plan that failed to set up may have entries without executor
      if (!action_info.action_executor) {
        continue;
      }
      // Also those still looking for a performer, or they start once it answers (#436)
      auto status = action_info.action_executor->get_internal_status();
      if (status == ActionExecutor::RUNNING || status == ActionExecutor::DEALING) {
        action_info.action_executor->cancel();
      }
    }
  }
}

void
ExecutorNode::restore_at_start_effects(std::map<std::string, ActionExecutionInfo> & action_map)
{
  for (auto & [action_id, info] : action_map) {
    restore_at_start_effects(action_id, info);
  }
}

void
ExecutorNode::restore_at_start_effects(const std::string & action_id, ActionExecutionInfo & info)
{
  if (!info.at_start_effects_applied || info.at_end_effects_applied ||
    !get_parameter("restore_at_start_effects").as_bool())
  {
    return;
  }

  RCLCPP_INFO(get_logger(), "Taking back the at start effects of %s", action_id.c_str());
  std::vector<std::string> kept;
  if (!undo(info.at_start_changes, problem_client_, &kept)) {
    RCLCPP_ERROR(
      get_logger(), "Could not take back all the at start effects of %s", action_id.c_str());
  }
  for (const auto & function : kept) {
    RCLCPP_WARN(
      get_logger(), "%s: %s changed since the action started, it keeps its value",
      action_id.c_str(), function.c_str());
  }
  info.at_start_changes.clear();
  info.at_start_effects_applied = false;
}

std::vector<plansys2_msgs::msg::ActionExecutionInfo>
ExecutorNode::get_feedback_info(
  std::shared_ptr<std::map<std::string,
  ActionExecutionInfo>> action_map)
{
  std::vector<plansys2_msgs::msg::ActionExecutionInfo> ret;

  if (!action_map) {
    return ret;
  }

  for (const auto & action : *action_map) {
    // ":0" is the INIT pseudo-action the BT starts from, not part of the plan (#436)
    if (action.first == ":0") {
      continue;
    }

    if (!action.second.action_executor) {
      // Executor not yet assigned (BT hasn't reached this action yet).
      // Still publish so subscribers know the action exists and is NOT_EXECUTED.
      plansys2_msgs::msg::ActionExecutionInfo info;
      info.status = plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED;
      info.action_full_name = action.first;
      info.action = action.second.plan_item.action;
      info.duration = rclcpp::Duration::from_seconds(action.second.duration);
      info.completion = 0.0;
      ret.push_back(info);
      continue;
    }

    plansys2_msgs::msg::ActionExecutionInfo info;
    switch (action.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
      case ActionExecutor::DEALING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED;
        break;
      case ActionExecutor::RUNNING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
        break;
      case ActionExecutor::SUCCESS:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED;
        break;
      case ActionExecutor::FAILURE:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::FAILED;
        break;
      case ActionExecutor::CANCELLED:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
        break;
    }

    info.action_full_name = action.first;

    info.start_stamp = action.second.action_executor->get_start_time();
    info.status_stamp = action.second.action_executor->get_status_time();
    if (action.second.at_start_effects_applied) {
      info.start_happening_stamp = action.second.at_start_effects_applied_time;
    }
    if (action.second.at_end_effects_applied) {
      info.end_happening_stamp = action.second.at_end_effects_applied_time;
    }
    info.action = action.second.action_executor->get_action_name();

    info.arguments = action.second.action_executor->get_action_params();
    info.duration = rclcpp::Duration::from_seconds(action.second.duration);
    info.completion = action.second.action_executor->get_completion();
    info.message_status = action.second.action_executor->get_feedback();

    ret.push_back(info);
  }

  return ret;
}

void
ExecutorNode::print_execution_info(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info)
{
  fprintf(stderr, "Execution info =====================\n");

  for (const auto & action_info : *exec_info) {
    fprintf(stderr, "[%s]", action_info.first.c_str());
    switch (action_info.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
        fprintf(stderr, "\tIDLE\n");
        break;
      case ActionExecutor::DEALING:
        fprintf(stderr, "\tDEALING\n");
        break;
      case ActionExecutor::RUNNING:
        fprintf(stderr, "\tRUNNING\n");
        break;
      case ActionExecutor::SUCCESS:
        fprintf(stderr, "\tSUCCESS\n");
        break;
      case ActionExecutor::FAILURE:
        fprintf(stderr, "\tFAILURE\n");
        break;
      case ActionExecutor::CANCELLED:
        fprintf(stderr, "\tCANCELLED\n");
        break;
    }
    if (action_info.second.action_info.is_empty()) {
      fprintf(stderr, "\tWith no action info\n");
    }

    if (action_info.second.at_start_effects_applied) {
      fprintf(stderr, "\tAt start effects applied\n");
    } else {
      fprintf(stderr, "\tAt start effects NOT applied\n");
    }

    if (action_info.second.at_end_effects_applied) {
      fprintf(stderr, "\tAt end effects applied\n");
    } else {
      fprintf(stderr, "\tAt end effects NOT applied\n");
    }
  }
}

void
ExecutorNode::update_plan(PlanRuntineInfo & runtime_info)
{
  for (const auto & action : *runtime_info.action_map) {
    if (action.second.action_executor == nullptr) {continue;}

    switch (action.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
      case ActionExecutor::DEALING:
      case ActionExecutor::RUNNING:
        break;
      case ActionExecutor::SUCCESS:
      case ActionExecutor::FAILURE:
      case ActionExecutor::CANCELLED:
        {
          auto pos = std::find(
            runtime_info.remaining_plan.items.begin(),
            runtime_info.remaining_plan.items.end(),
            action.second.plan_item);
          if (pos != runtime_info.remaining_plan.items.end()) {
            runtime_info.remaining_plan.items.erase(pos);
          }
        }
        break;
    }
  }
}

rclcpp_action::GoalResponse
ExecutorNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request with order");

  // Nothing would execute it (#435)
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "Rejecting plan: executor is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ExecutorNode::handle_cancel(
  const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  // execution_cycle sees is_canceling() on the goal, running or waiting (#436)
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ExecutorNode::domain_topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  if (!domain_baseline_seen_) {
    domain_baseline_seen_ = true;
    return;
  }

  // executor_state_ belongs to execution_cycle's thread: only flag the change here
  domain_changed_ = true;
}

void
ExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Accepted new goal");

  std::lock_guard<std::mutex> lock(goal_mutex_);
  // A goal still waiting to start is replaced: its client must not wait forever (#436)
  if (new_plan_received_) {
    auto result = std::make_shared<ExecutePlan::Result>();
    result->result = plansys2_msgs::action::ExecutePlan::Result::PREEMPT;
    finish_replaced_goal(new_goal_handle_, result);
  }
  new_goal_handle_ = goal_handle;
  new_plan_received_ = true;
  std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
  starting_goal_ = goal_handle;
  starting_plan_ = goal_handle->get_goal()->plan;
}

void
ExecutorNode::starting_done(const std::shared_ptr<GoalHandleExecutePlan> & goal)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (starting_goal_ == goal) {
    starting_goal_.reset();
  }
}

namespace
{
// When it goes out of scope, however the start of a goal ended
template<typename F>
struct AtExit
{
  F f;
  ~AtExit() {f();}
};
template<typename F>
AtExit<F> at_exit(F f) {return AtExit<F>{f};}
}  // namespace

std::shared_ptr<ExecutorNode::GoalHandleExecutePlan>
ExecutorNode::take_new_goal()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!new_plan_received_.exchange(false)) {
    return nullptr;
  }
  auto goal = std::move(new_goal_handle_);
  new_goal_handle_ = nullptr;
  return goal;
}

void
ExecutorNode::finish_replaced_goal(
  const std::shared_ptr<GoalHandleExecutePlan> & goal,
  const std::shared_ptr<ExecutePlan::Result> & result)
{
  if (!goal || !goal->is_active()) {
    return;
  }
  if (goal->is_canceling()) {
    goal->canceled(result);
  } else {
    goal->abort(result);
  }
}

std::chrono::nanoseconds
ExecutorNode::feedback_period()
{
  double rate = get_parameter("feedback_rate").as_double();
  if (!(rate > 0.0)) {
    rate = 5.0;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate));
}

void
ExecutorNode::execution_cycle()
{
  rclcpp::Rate rate(50);
  while (rclcpp::ok() && node_running_) {
    try {
      execution_step();
    } catch (const std::exception & e) {
      // Publishing or finishing a goal throws once rclcpp shuts down (Ctrl-C): just stop
      if (!rclcpp::ok()) {
        break;
      }
      RCLCPP_ERROR(get_logger(), "Plan execution failed: %s", e.what());
      fail_current_goals();
    }
    rate.sleep();
  }
}

void
ExecutorNode::execution_step()
{
  make_replanning_problem();

  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  switch (executor_state_) {
    case STATE_IDLE:
      {
        // Nothing runs: nothing to take back for a replanning problem
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        running_changes_snapshot_.clear();
      }
      if (auto goal = take_new_goal()) {
        auto started = at_exit([this, goal]() {starting_done(goal);});
        // A domain change before this plan started does not affect it
        domain_changed_ = false;

        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          current_goal_handle_ = goal;
        }

        // Cancelled while waiting to start (#436)
        if (current_goal_handle_->is_canceling()) {
          result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
          current_goal_handle_->canceled(result);
          break;
        }

        if (current_goal_handle_->get_goal()->plan.items.empty()) {
          // Nothing to do (#431)
          result->result = plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
          current_goal_handle_->succeed(result);
          break;
        }

        runtime_info_ = PlanRuntineInfo();
        runtime_info_.complete_plan = current_goal_handle_->get_goal()->plan;
        last_executing_plan_.reset();  // A new goal publishes its plan
        runtime_info_.remaining_plan = current_goal_handle_->get_goal()->plan;

        if (!init_plan_for_execution(runtime_info_)) {
          executor_state_ = STATE_ABORTING;
        } else {
          update_snapshot();
          executor_state_ = STATE_EXECUTING;
        }
      }
      break;
    case STATE_EXECUTING:
      {
        BT::NodeStatus status = BT::NodeStatus::FAILURE;
        if (runtime_info_.tree_done) {
          // PDDL+: the actions are done; the plan ends at its end, with its goal holding
          status = BT::NodeStatus::RUNNING;
          // Shifted by the actions that started late
          std::ignore = runtime_info_.current_tree->tree.rootBlackboard()->get(
            "plan_start", runtime_info_.plan_start);
          const double elapsed = (now() - runtime_info_.plan_start).seconds();
          if (elapsed >= runtime_info_.complete_plan.end) {
            const double deadline = runtime_info_.complete_plan.end +
              get_parameter("plan_end_goal_timeout").as_double();
            if (problem_client_->isGoalSatisfied(problem_client_->getGoal())) {
              status = BT::NodeStatus::SUCCESS;
            } else if (elapsed > deadline) {
              RCLCPP_ERROR(
                get_logger(), "[%s] The goal does not hold at the end of the plan (%.3f s)",
                get_name(), runtime_info_.complete_plan.end);
              status = BT::NodeStatus::FAILURE;
            }
          }
        } else {
          try {
            status = runtime_info_.current_tree->tree.tickOnce();
          } catch (std::exception & e) {
            std::cerr << e.what() << std::endl;
            executor_state_ = STATE_FAILED;
          }
          if (status == BT::NodeStatus::SUCCESS && runtime_info_.complete_plan.has_end) {
            runtime_info_.tree_done = true;
            status = BT::NodeStatus::RUNNING;
          }
        }

        if (status != BT::NodeStatus::FAILURE) {
          if (auto violated = check_constraints(status)) {
            RCLCPP_ERROR(
              get_logger(), "[%s] Constraint violated: %s", get_name(), violated->c_str());
            trace_->event(ExecutionTrace::Event::VIOLATION, "", *violated);
            status = BT::NodeStatus::FAILURE;
          }
        }

        update_plan(runtime_info_);
        update_snapshot();

        // Latched: once per plan
        if (!last_executing_plan_ || *last_executing_plan_ != runtime_info_.complete_plan) {
          executing_plan_pub_->publish(runtime_info_.complete_plan);
          last_executing_plan_ = runtime_info_.complete_plan;
        }

        // The tree ticks at 50 Hz; what is published for others goes at feedback_rate, and
        // when the plan ends, so that its last state is seen
        const auto now = std::chrono::steady_clock::now();
        const bool ending = status != BT::NodeStatus::RUNNING;
        if (ending || now >= next_feedback_time_) {
          next_feedback_time_ = now + feedback_period();

          auto feedback_info_msgs = get_feedback_info(runtime_info_.action_map);
          feedback->action_execution_status = feedback_info_msgs;
          // rclcpp_action only takes feedback from executing goals
          if (!current_goal_handle_->is_canceling()) {
            current_goal_handle_->publish_feedback(feedback);
          }
          for (const auto & msg : feedback_info_msgs) {
            execution_info_pub_->publish(msg);
          }
          remaining_plan_pub_->publish(runtime_info_.remaining_plan);

          // Building it costs: only for someone listening
          if (dotgraph_pub_->get_subscription_count() > 0) {
            std_msgs::msg::String dotgraph_msg;
            dotgraph_msg.data = runtime_info_.current_tree->bt_builder->get_dotgraph(
              runtime_info_.action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
              this->get_parameter("print_graph").as_bool());
            dotgraph_pub_->publish(dotgraph_msg);
          }
        }

        if (status == BT::NodeStatus::SUCCESS) {
          executor_state_ = STATE_SUCCEDED;
        } else if (status == BT::NodeStatus::FAILURE) {
          executor_state_ = STATE_FAILED;
        } else if (domain_changed_.exchange(false)) {
          RCLCPP_WARN(
            get_logger(),
            "[%s] Domain changed while executing a plan, cancelling execution", get_name());
          executor_state_ = STATE_ABORTING;
        } else if (current_goal_handle_->is_canceling()) {
          // Checked on the goal itself, so a late cancel never leaks to the next one (#436)
          executor_state_ = STATE_CANCELLED;
        } else if (new_plan_received_) {
          executor_state_ = STATE_REPLANNING;
        }
      }
      break;
    case STATE_REPLANNING:
      {
        auto goal = take_new_goal();
        if (!goal) {
          executor_state_ = STATE_EXECUTING;
          break;
        }
        auto started = at_exit([this, goal]() {starting_done(goal);});

        // The running goal is replaced by the new one
        result->result = plansys2_msgs::action::ExecutePlan::Result::PREEMPT;
        result->action_execution_status = get_feedback_info(runtime_info_.action_map);
        finish_replaced_goal(current_goal_handle_, result);

        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          current_goal_handle_ = goal;
        }

        // Cancelled while waiting to start (#436)
        if (current_goal_handle_->is_canceling()) {
          executor_state_ = STATE_CANCELLED;
          break;
        }

        runtime_info_.complete_plan = current_goal_handle_->get_goal()->plan;
        last_executing_plan_.reset();  // A new goal publishes its plan
        runtime_info_.remaining_plan = current_goal_handle_->get_goal()->plan;

        if (!replan_for_execution(runtime_info_)) {
          executor_state_ = STATE_ERROR;
        } else {
          update_snapshot();
          executor_state_ = STATE_EXECUTING;
        }
      }
      break;
    case STATE_ABORTING:
      stop_continuous_effects(runtime_info_);
      cancel_all_running_actions(runtime_info_);

      result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
      result->action_execution_status = get_feedback_info(runtime_info_.action_map);
      if (runtime_info_.action_map) {
        restore_at_start_effects(*runtime_info_.action_map);
      }

      current_goal_handle_->abort(result);
      trace_->event(ExecutionTrace::Event::PLAN_END, "", "aborted");
      executor_state_ = STATE_IDLE;
      break;
    case STATE_CANCELLED:
      stop_continuous_effects(runtime_info_);
      cancel_all_running_actions(runtime_info_);

      result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
      result->action_execution_status = get_feedback_info(runtime_info_.action_map);
      if (runtime_info_.action_map) {
        restore_at_start_effects(*runtime_info_.action_map);
      }

      current_goal_handle_->canceled(result);
      trace_->event(ExecutionTrace::Event::PLAN_END, "", "cancelled");
      executor_state_ = STATE_IDLE;
      break;
    case STATE_FAILED:
      stop_continuous_effects(runtime_info_);
      cancel_all_running_actions(runtime_info_);

      result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
      result->action_execution_status = get_feedback_info(runtime_info_.action_map);
      if (runtime_info_.action_map) {
        restore_at_start_effects(*runtime_info_.action_map);
      }

      // A failed plan is not a succeeded goal (#436)
      current_goal_handle_->abort(result);
      trace_->event(ExecutionTrace::Event::PLAN_END, "", "failed");
      executor_state_ = STATE_IDLE;
      break;
    case STATE_ERROR:
      stop_continuous_effects(runtime_info_);
      cancel_all_running_actions(runtime_info_);

      result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
      result->action_execution_status = get_feedback_info(runtime_info_.action_map);
      if (runtime_info_.action_map) {
        restore_at_start_effects(*runtime_info_.action_map);
      }

      current_goal_handle_->abort(result);
      trace_->event(ExecutionTrace::Event::PLAN_END, "", "error setting up the new plan");
      executor_state_ = STATE_IDLE;
      break;
    case STATE_SUCCEDED:
      stop_continuous_effects(runtime_info_);
      result->result = plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
      result->action_execution_status = get_feedback_info(runtime_info_.action_map);

      current_goal_handle_->succeed(result);
      trace_->event(ExecutionTrace::Event::PLAN_END, "", "succeeded");
      executor_state_ = STATE_IDLE;
      break;
  }
}

void
ExecutorNode::update_snapshot()
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  complete_plan_snapshot_ = runtime_info_.complete_plan;
  remaining_plan_snapshot_ = runtime_info_.remaining_plan;
  ordered_sub_goals_snapshot_ = runtime_info_.ordered_sub_goals;
  running_changes_snapshot_.clear();
  if (runtime_info_.action_map) {
    for (const auto & [id, info] : *runtime_info_.action_map) {
      (void)id;
      if (info.at_start_effects_applied && !info.at_end_effects_applied) {
        running_changes_snapshot_.insert(
          running_changes_snapshot_.end(), info.at_start_changes.begin(),
          info.at_start_changes.end());
      }
    }
  }
}

void
ExecutorNode::start_execution_thread()
{
  if (execution_thread_.joinable()) {
    return;
  }
  node_running_ = true;
  execution_thread_ = std::thread(&ExecutorNode::execution_cycle, this);
}

void
ExecutorNode::stop_execution_thread()
{
  node_running_ = false;
  if (execution_thread_.joinable()) {
    // Its calls must not wait for a component that may be gone (or stopping too)
    set_clients_interrupted(true);
    execution_thread_.join();
    set_clients_interrupted(false);
  }

  // Nothing executes plans from now on: fail the goals in progress or waiting
  fail_current_goals();
}

void
ExecutorNode::set_clients_interrupted(bool interrupted)
{
  if (domain_client_) {
    domain_client_->interrupt(interrupted);
  }
  if (problem_client_) {
    problem_client_->interrupt(interrupted);
  }
  if (planner_client_) {
    planner_client_->interrupt(interrupted);
  }
}

void
ExecutorNode::fail_current_goals()
{
  auto result = std::make_shared<ExecutePlan::Result>();
  result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
  // After shutdown (e.g. from the destructor) goal handles throw: nobody would get the result
  try {
    if (executor_state_ != STATE_IDLE && current_goal_handle_ &&
      current_goal_handle_->is_active())
    {
      cancel_all_running_actions(runtime_info_);
      result->action_execution_status = get_feedback_info(runtime_info_.action_map);
      current_goal_handle_->abort(result);
    }
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Failed to abort the plan in execution: %s", e.what());
    }
  }
  executor_state_ = STATE_IDLE;

  std::lock_guard<std::mutex> lock(goal_mutex_);
  try {
    if (new_plan_received_ && new_goal_handle_ && new_goal_handle_->is_active()) {
      result->action_execution_status.clear();
      new_goal_handle_->abort(result);
    }
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Failed to abort the waiting plan: %s", e.what());
    }
  }
  new_plan_received_ = false;
  std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
  starting_goal_.reset();
}

void ExecutorNode::add_groot_monitoring(BT::Tree * tree, uint16_t server_port)
{
  // This logger publish status changes using Groot2
  groot_monitor_ = std::make_unique<BT::Groot2Publisher>(*tree, server_port);

  // Register common types JSON definitions
  BT::RegisterJsonDefinition<builtin_interfaces::msg::Time>();
  BT::RegisterJsonDefinition<std_msgs::msg::Header>();
}

void ExecutorNode::reset_groot_monitor()
{
  if (groot_monitor_) {
    groot_monitor_.reset();
  }
}

int
ExecutorNode::action_time_precision()
{
  // One precision for every action id: the builders and the action map must agree
  const auto precision = get_parameter("action_time_precision").as_int();
  if (precision < 0 || precision > 9) {
    RCLCPP_WARN_ONCE(
      get_logger(), "action_time_precision must be between 0 and 9 (%ld), using 3", precision);
    return 3;
  }
  return static_cast<int>(precision);
}

}  // namespace plansys2
