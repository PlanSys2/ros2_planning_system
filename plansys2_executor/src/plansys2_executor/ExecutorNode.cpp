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
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <vector>

#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "ament_index_cpp/get_package_share_path.hpp"

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
  this->declare_parameter<bool>("enable_dotgraph_legend", true);
  this->declare_parameter<bool>("print_graph", false);
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

  execute_plan_action_server_ = rclcpp_action::create_server<ExecutePlan>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "execute_plan",
    std::bind(&ExecutorNode::handle_goal, this, _1, _2),
    std::bind(&ExecutorNode::handle_cancel, this, _1),
    std::bind(&ExecutorNode::handle_accepted, this, _1));

  get_ordered_sub_goals_service_ = create_service<plansys2_msgs::srv::GetOrderedSubGoals>(
    "executor/get_ordered_sub_goals",
    std::bind(
      &ExecutorNode::get_ordered_sub_goals_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
    "executor/get_plan",
    std::bind(
      &ExecutorNode::get_plan_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_remaining_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
    "executor/get_remaining_plan",
    std::bind(
      &ExecutorNode::get_remaining_plan_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
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
    auto pkg_path = ament_index_cpp::get_package_share_path("plansys2_executor");
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
    auto pkg_path = ament_index_cpp::get_package_share_path("plansys2_executor");
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
    auto pkg_path = ament_index_cpp::get_package_share_path("plansys2_executor");
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

  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();

  // A new subscription gets the latched domain again: that is not a change
  domain_baseline_seen_ = false;
  domain_sub_ = create_subscription<std_msgs::msg::String>(
    "domain_expert/domain",
    rclcpp::QoS(100).transient_local(),
    std::bind(&ExecutorNode::domain_topic_callback, this, std::placeholders::_1));

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

  for (const auto & plan_item : runtime_info.complete_plan.items) {
    auto actions = domain_client_->getActions();
    std::string action_name = get_action_name(plan_item.action);
    if (std::find(actions.begin(), actions.end(), action_name) != actions.end()) {
      std::shared_ptr<plansys2_msgs::msg::Action> action =
        domain_client_->getAction(
        action_name, get_action_params(plan_item.action));
      apply(action->effects, local_predicates, local_functions);
    } else {
      std::shared_ptr<plansys2_msgs::msg::DurativeAction> action =
        domain_client_->getDurativeAction(
        action_name, get_action_params(plan_item.action));
      apply(action->at_start_effects, local_predicates, local_functions);
      apply(action->at_end_effects, local_predicates, local_functions);
    }


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
  if (executor_state_ == STATE_EXECUTING) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

  for (const auto & plan_item : runtime_info.complete_plan.items) {
    // The parsing below assumes "(name args...)"; anything else must not reach it
    const auto & action = plan_item.action;
    if (action.size() < 3 || action.front() != '(' || action.back() != ')' ||
      action.find_first_not_of(" \t()") == std::string::npos)
    {
      throw std::runtime_error("malformed action in plan: [" + action + "]");
    }

    auto index = BTBuilder::to_action_id(plan_item, 3);
    (*runtime_info.action_map)[index] = ActionExecutionInfo();
    (*runtime_info.action_map)[index].plan_item = plan_item;
    (*runtime_info.action_map)[index].action_executor =
      ActionExecutor::make_shared(plan_item.action, non_owning_this());

    auto actions = domain_client_->getActions();
    std::string action_name = get_action_name(plan_item.action);
    if (std::find(actions.begin(), actions.end(), action_name) != actions.end()) {
      (*runtime_info.action_map)[index].action_info = domain_client_->getAction(
        action_name, get_action_params(plan_item.action));
    } else {
      (*runtime_info.action_map)[index].action_info = domain_client_->getDurativeAction(
        action_name, get_action_params(plan_item.action));
    }
    if ((*runtime_info.action_map)[index].action_info.is_empty()) {
      throw std::runtime_error("action not in the domain: " + plan_item.action);
    }

    action_name = (*runtime_info.action_map)[index].action_info.get_action_name();
    (*runtime_info.action_map)[index].duration = plan_item.duration;

    if (std::find(
        action_timeout_actions.begin(), action_timeout_actions.end(),
        action_name) != action_timeout_actions.end() &&
      this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage"))
    {
      (*runtime_info.action_map)[index].duration_overrun_percentage = this->get_parameter(
        "action_timeouts." + action_name + ".duration_overrun_percentage").as_double();
    }
    RCLCPP_INFO(
      get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
      (*runtime_info.action_map)[index].duration_overrun_percentage);
  }

  runtime_info.ordered_sub_goals = {};
  get_ordered_subgoals(runtime_info);
}

bool
ExecutorNode::get_tree_from_plan(PlanRuntineInfo & runtime_info)
{
  auto bt_builder_plugin = this->get_parameter("bt_builder_plugin").as_string();
  if (bt_builder_plugin.empty()) {
    bt_builder_plugin = "SimpleBTBuilder";
  }

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  try {
    bt_builder = bt_builder_loader_.createSharedInstance("plansys2::" + bt_builder_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());
    return false;
  }

  if (bt_builder_plugin == "STNBTBuilder") {
    RCLCPP_WARN(get_logger(), "STN disabled until fixed. Using SimpleBTBuilder instead");
    bt_builder = bt_builder_loader_.createSharedInstance("plansys2::SimpleBTBuilder");
    // auto precision = this->get_parameter("action_time_precision").as_int();
    // bt_builder->initialize(start_action_bt_xml_, end_action_bt_xml_, precision);
  }
  // Every builder needs the action BT template (#431)
  bt_builder->initialize(action_bt_xml_);

  auto bt_xml_tree = bt_builder->get_tree(runtime_info.complete_plan);
  if (bt_xml_tree.empty()) {
    RCLCPP_ERROR(get_logger(), "Error computing behavior tree!");
    return false;
  }

  auto action_graph = bt_builder->get_graph();
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
    runtime_info.action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
    this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  std::filesystem::path tp = std::filesystem::temp_directory_path();
  std::ofstream out(std::string("/tmp/") + get_namespace() + "/bt.xml");
  out << bt_xml_tree;
  out.close();

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

bool
ExecutorNode::replan_for_execution_impl(PlanRuntineInfo & runtime_info)
{
  cancel_plan_requested_ = false;
  replan_requested_ = false;

  std::map<std::string, ActionExecutionInfo> previous_action_map = *runtime_info.action_map;

  bool plan_success = false;
  try {
    create_plan_runtime_info(runtime_info);
    plan_success = get_tree_from_plan(runtime_info);
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
      match_it->second.action_executor = action_info.second.action_executor;
      match_it->second.at_start_effects_applied = action_info.second.at_start_effects_applied;
      match_it->second.at_end_effects_applied = action_info.second.at_end_effects_applied;
      match_it->second.at_start_effects_applied_time =
        action_info.second.at_start_effects_applied_time;
      match_it->second.execution_error_info = action_info.second.execution_error_info;
      match_it->second.duration = action_info.second.duration;
      match_it->second.duration_overrun_percentage =
        action_info.second.duration_overrun_percentage;
    } else {
      action_info.second.action_executor->cancel();
      action_info.second.action_executor->clean_up();
      action_info.second.action_executor = nullptr;
    }
  }

  if (!plan_success) {
    return false;
  }

  return true;
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
}

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

void
ExecutorNode::execution_cycle()
{
  rclcpp::Rate rate(50);
  while (rclcpp::ok() && node_running_) {
    auto feedback = std::make_shared<ExecutePlan::Feedback>();
    auto result = std::make_shared<ExecutePlan::Result>();

    switch (executor_state_) {
      case STATE_IDLE:
        if (auto goal = take_new_goal()) {
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
          try {
            status = runtime_info_.current_tree->tree.tickOnce();
          } catch (std::exception & e) {
            std::cerr << e.what() << std::endl;
            executor_state_ = STATE_FAILED;
          }

          auto feedback_info_msgs = get_feedback_info(runtime_info_.action_map);
          feedback->action_execution_status = feedback_info_msgs;
          // rclcpp_action only takes feedback from executing goals
          if (!current_goal_handle_->is_canceling()) {
            current_goal_handle_->publish_feedback(feedback);
          }
          for (const auto & msg : feedback_info_msgs) {
            execution_info_pub_->publish(msg);
          }

          update_plan(runtime_info_);
          update_snapshot();
          remaining_plan_pub_->publish(runtime_info_.remaining_plan);
          executing_plan_pub_->publish(runtime_info_.complete_plan);

          std_msgs::msg::String dotgraph_msg;
          dotgraph_msg.data = runtime_info_.current_tree->bt_builder->get_dotgraph(
            runtime_info_.action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
            this->get_parameter("print_graph").as_bool());
          dotgraph_pub_->publish(dotgraph_msg);

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
        cancel_all_running_actions(runtime_info_);

        result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
        result->action_execution_status = get_feedback_info(runtime_info_.action_map);

        current_goal_handle_->abort(result);
        executor_state_ = STATE_IDLE;
        break;
      case STATE_CANCELLED:
        cancel_all_running_actions(runtime_info_);

        result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
        result->action_execution_status = get_feedback_info(runtime_info_.action_map);

        current_goal_handle_->canceled(result);
        executor_state_ = STATE_IDLE;
        break;
      case STATE_FAILED:
        cancel_all_running_actions(runtime_info_);

        result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
        result->action_execution_status = get_feedback_info(runtime_info_.action_map);

        // A failed plan is not a succeeded goal (#436)
        current_goal_handle_->abort(result);
        executor_state_ = STATE_IDLE;
        break;
      case STATE_ERROR:
        cancel_all_running_actions(runtime_info_);

        result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
        result->action_execution_status = get_feedback_info(runtime_info_.action_map);

        current_goal_handle_->abort(result);
        executor_state_ = STATE_IDLE;
        break;
      case STATE_SUCCEDED:
        result->result = plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
        result->action_execution_status = get_feedback_info(runtime_info_.action_map);

        current_goal_handle_->succeed(result);
        executor_state_ = STATE_IDLE;
        break;
    }

    rate.sleep();
  }
}

void
ExecutorNode::update_snapshot()
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  complete_plan_snapshot_ = runtime_info_.complete_plan;
  remaining_plan_snapshot_ = runtime_info_.remaining_plan;
  ordered_sub_goals_snapshot_ = runtime_info_.ordered_sub_goals;
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
    execution_thread_.join();
  }

  // Nothing executes plans from now on: fail the goals in progress or waiting
  auto result = std::make_shared<ExecutePlan::Result>();
  result->result = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
  if (executor_state_ != STATE_IDLE && current_goal_handle_ && current_goal_handle_->is_active()) {
    cancel_all_running_actions(runtime_info_);
    result->action_execution_status = get_feedback_info(runtime_info_.action_map);
    current_goal_handle_->abort(result);
  }
  executor_state_ = STATE_IDLE;

  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (new_plan_received_ && new_goal_handle_ && new_goal_handle_->is_active()) {
    result->action_execution_status.clear();
    new_goal_handle_->abort(result);
  }
  new_plan_received_ = false;
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

}  // namespace plansys2
