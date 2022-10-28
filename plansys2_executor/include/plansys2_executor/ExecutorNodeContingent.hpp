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

#ifndef PLANSYS2_EXECUTOR__EXECUTORNODECONTINGENT_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORNODECONTINGENT_HPP_

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
#include "plansys2_pddl_parser/Utils.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/check_action_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/apply_observation_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_timeout_node.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"

#include "ExecutorNodeBase.hpp"

namespace plansys2 {

  class ExecutorNodeContingent : public ExecutorNodeBase {
  public:
    using ExecutePlan = plansys2_msgs::action::ExecutePlan;
    using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;

    ExecutorNodeContingent();

  protected:

    void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle) override;

    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ExecutePlan::Goal> goal) override;

  };

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODECONTINGENT_HPP_
