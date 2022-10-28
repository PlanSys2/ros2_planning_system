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

#include "plansys2_executor/ExecutorNode.hpp"


namespace plansys2
{

using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using namespace std::chrono_literals;

ExecutorNode::ExecutorNode() : ExecutorNodeBase()
{
  using namespace std::placeholders;

  get_ordered_sub_goals_service_ = create_service<plansys2_msgs::srv::GetOrderedSubGoals>(
    "executor/get_ordered_sub_goals",
    std::bind(
      &ExecutorNode::get_ordered_sub_goals_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
}

void
ExecutorNode::get_ordered_sub_goals_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response)
{
  if (ordered_sub_goals_.has_value()) {
    response->sub_goals = ordered_sub_goals_.value();
    response->success = true;
  } else {
    response->success = false;
    response->error_info = "No current plan.";
  }
}

std::optional<std::vector<plansys2_msgs::msg::Tree>>
ExecutorNode::getOrderedSubGoals()
{
  if (!current_plan_.has_value()) {
    return {};
  }

  auto goal = problem_client_->getGoal();
  auto local_predicates = problem_client_->getPredicates();
  auto local_functions = problem_client_->getFunctions();

  std::vector<plansys2_msgs::msg::Tree> ordered_goals;
  std::vector<uint32_t> unordered_subgoals = parser::pddl::getSubtreeIds(goal);

  // just in case some goals are already satisfied
  for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end(); ) {
    if (check(goal, local_predicates, local_functions, *it)) {
      plansys2_msgs::msg::Tree new_goal;
      parser::pddl::fromString(new_goal, "(and " + parser::pddl::toString(goal, (*it)) + ")");
      ordered_goals.push_back(new_goal);
      it = unordered_subgoals.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto & plan_item : current_plan_.value().items) {
    std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_action =
    domain_client_->getDurativeAction(
    get_action_name(plan_item.action), get_action_params(plan_item.action));
    if (durative_action){
      apply(durative_action->at_start_effects, local_predicates, local_functions);
      apply(durative_action->at_end_effects, local_predicates, local_functions);
    } else{
      std::shared_ptr<plansys2_msgs::msg::Action> action = domain_client_->getAction(
      get_action_name(plan_item.action), get_action_params(plan_item.action));
      apply(action->effects, local_predicates, local_functions);
    }


    for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end(); ) {
      if (check(goal, local_predicates, local_functions, *it)) {
        plansys2_msgs::msg::Tree new_goal;
        parser::pddl::fromString(new_goal, "(and " + parser::pddl::toString(goal, (*it)) + ")");
        ordered_goals.push_back(new_goal);
        it = unordered_subgoals.erase(it);
      } else {
        ++it;
      }
    }
  }

  return ordered_goals;
}

rclcpp_action::GoalResponse
ExecutorNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  RCLCPP_DEBUG(this->get_logger(), "Received goal request with order");

  current_plan_ = {};
  ordered_sub_goals_ = {};

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void
ExecutorNode::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  cancel_plan_requested_ = false;

  current_plan_ = goal_handle->get_goal()->plan;

  if (!current_plan_.has_value()) {
    RCLCPP_ERROR(get_logger(), "No plan found");
    result->success = false;
    goal_handle->succeed(result);

    // Publish void plan
    executing_plan_pub_->publish(plansys2_msgs::msg::Plan());
    return;
  }

  executing_plan_pub_->publish(current_plan_.value());

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();
  auto action_timeout_actions = this->get_parameter("action_timeouts.actions").as_string_array();

  for (const auto & plan_item : current_plan_.value().items) {
    auto index = BTBuilder::to_action_id(plan_item, 3);

    auto durative_action_info = domain_client_->getDurativeAction(get_action_name(plan_item.action), get_action_params(plan_item.action));
    if (durative_action_info){
      (*action_map)[index] = ActionExecutionInfo();
      (*action_map)[index].action_executor =
          ActionExecutor::make_shared(plan_item.action, shared_from_this());
      (*action_map)[index].durative_action_info = durative_action_info;
      (*action_map)[index].duration = plan_item.duration;
      std::string action_name = (*action_map)[index].durative_action_info->name;
      if (std::find(
          action_timeout_actions.begin(), action_timeout_actions.end(),
          action_name) != action_timeout_actions.end() &&
          this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage"))
      {
        (*action_map)[index].duration_overrun_percentage = this->get_parameter(
            "action_timeouts." + action_name + ".duration_overrun_percentage").as_double();
      }
      RCLCPP_INFO(
          get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
          (*action_map)[index].duration_overrun_percentage);
    }

    auto action_info = domain_client_->getAction(get_action_name(plan_item.action), get_action_params(plan_item.action));
    if (action_info) { // TODO support both action types??
      (*action_map)[index] = ActionExecutionInfo();
      (*action_map)[index].action_executor = ActionExecutor::make_shared(plan_item.action, shared_from_this());
      (*action_map)[index].durative_action_info = std::make_shared<plansys2_msgs::msg::DurativeAction>();
      (*action_map)[index].durative_action_info->name = action_info->name;
      (*action_map)[index].durative_action_info->parameters = action_info->parameters;
      (*action_map)[index].durative_action_info->observe = action_info->observe;
      (*action_map)[index].durative_action_info->at_start_requirements = action_info->preconditions;
      (*action_map)[index].durative_action_info->at_end_effects = action_info->effects;
      (*action_map)[index].duration = plan_item.duration;
      std::string action_name = action_info->name;
      if (std::find(
          action_timeout_actions.begin(), action_timeout_actions.end(),
          action_name) != action_timeout_actions.end() &&
          this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage"))
      {
        (*action_map)[index].duration_overrun_percentage = this->get_parameter(
            "action_timeouts." + action_name + ".duration_overrun_percentage").as_double();
      }
      RCLCPP_INFO(
          get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
          (*action_map)[index].duration_overrun_percentage);
    }

  }

  ordered_sub_goals_ = getOrderedSubGoals();

  auto bt_builder_plugin = this->get_parameter("bt_builder_plugin").as_string();
  if (bt_builder_plugin.empty()) {
    bt_builder_plugin = "SimpleBTBuilder";
  }

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  try {
    bt_builder = bt_builder_loader_.createSharedInstance("plansys2::" + bt_builder_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());
  }

  if (bt_builder_plugin == "SimpleBTBuilder") {
    bt_builder->initialize(action_bt_xml_);
  } else if (bt_builder_plugin == "STNBTBuilder") {
    auto precision = this->get_parameter("action_time_precision").as_int();
    bt_builder->initialize(start_action_bt_xml_, end_action_bt_xml_, precision);
  } else if (bt_builder_plugin == "ContingentBTBuilder") {
    bt_builder->initialize();
  }
  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", shared_from_this());
  blackboard->set("domain_client", domain_client_);
  blackboard->set("problem_client", problem_client_);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ApplyAtEndEffect>("ApplyAtEndEffect");
  factory.registerNodeType<ApplyAtStartEffect>("ApplyAtStartEffect");
  factory.registerNodeType<CheckAction>("CheckAction");
  factory.registerNodeType<CheckAtEndReq>("CheckAtEndReq");
  factory.registerNodeType<ApplyObservation>("ApplyObservation");
  factory.registerNodeType<CheckOverAllReq>("CheckOverAllReq");
  factory.registerNodeType<CheckTimeout>("CheckTimeout");
  factory.registerNodeType<ExecuteAction>("ExecuteAction");
  factory.registerNodeType<WaitAction>("WaitAction");
  factory.registerNodeType<WaitAtStartReq>("WaitAtStartReq");

  auto bt_xml_tree = bt_builder->get_tree(current_plan_.value());
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
    action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
    this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  std::filesystem::path tp = std::filesystem::temp_directory_path();
  std::ofstream out(std::string("/tmp/") + get_namespace() + "/bt.xml");
  out << bt_xml_tree;
  out.close();

  auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

#ifdef ZMQ_FOUND
  unsigned int publisher_port = this->get_parameter("publisher_port").as_int();
  unsigned int server_port = this->get_parameter("server_port").as_int();
  unsigned int max_msgs_per_second = this->get_parameter("max_msgs_per_second").as_int();

  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
  if (this->get_parameter("enable_groot_monitoring").as_bool()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[%s] Groot monitoring: Publisher port: %d, Server port: %d, Max msgs per second: %d",
      get_name(), publisher_port, server_port, max_msgs_per_second);
    try {
      publisher_zmq.reset(
        new BT::PublisherZMQ(
          tree, max_msgs_per_second, publisher_port,
          server_port));
    } catch (const BT::LogicError & exc) {
      RCLCPP_ERROR(get_logger(), "ZMQ error: %s", exc.what());
    }
  }
#endif

  auto info_pub = create_wall_timer(
    1s, [this, &action_map]() {
      auto msgs = get_feedback_info(action_map);
      for (const auto & msg : msgs) {
        execution_info_pub_->publish(msg);
      }
    });

  rclcpp::Rate rate(10);
  auto status = BT::NodeStatus::RUNNING;

  while (status == BT::NodeStatus::RUNNING && !cancel_plan_requested_) {
    try {
      status = tree.tickRoot();
    } catch (std::exception & e) {
      std::cerr << e.what() << std::endl;
      status == BT::NodeStatus::FAILURE;
    }

    feedback->action_execution_status = get_feedback_info(action_map);
    goal_handle->publish_feedback(feedback);

    dotgraph_msg.data = bt_builder->get_dotgraph(
      action_map, this->get_parameter("enable_dotgraph_legend").as_bool());
    dotgraph_pub_->publish(dotgraph_msg);

    rate.sleep();
  }

  if (cancel_plan_requested_) {
    tree.haltTree();
  }

  if (status == BT::NodeStatus::FAILURE) {
    tree.haltTree();
    RCLCPP_ERROR(get_logger(), "Executor BT finished with FAILURE state");
  }

  dotgraph_msg.data = bt_builder->get_dotgraph(
    action_map, this->get_parameter("enable_dotgraph_legend").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  result->success = status == BT::NodeStatus::SUCCESS;
  result->action_execution_status = get_feedback_info(action_map);

  // TODO only the output of the BT matters, this doesn't make sense
  //  size_t i = 0;
  //  while (i < result->action_execution_status.size() && result->success) {
  //    if (result->action_execution_status[i].status ==
  //      plansys2_msgs::msg::ActionExecutionInfo::FAILED ||
  //        result->action_execution_status[i].status ==
  //        plansys2_msgs::msg::ActionExecutionInfo::CANCELLED)
  //    {
  //      result->success = false;
  //    }
  //    i++;
  //  }

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Plan Succeeded");
    } else {
      RCLCPP_INFO(this->get_logger(), "Plan Failed");
    }
  }
}


}  // namespace plansys2
