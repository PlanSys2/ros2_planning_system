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

#include <memory>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_msgs/action/execute_plan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

class ExecutorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ExecutorNode();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

private:
  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_plan_action_server_;

  rclcpp::Node::SharedPtr node_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle);
  
  void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
