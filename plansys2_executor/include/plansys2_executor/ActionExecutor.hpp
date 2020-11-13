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

#include <string>
#include <memory>
#include <vector>

#include "std_msgs/msg/empty.hpp"
#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/Types.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

class ActionExecutor
{
public:
  using ExecuteAction = plansys2_msgs::action::ExecuteAction;
  using GoalHandleExecuteAction = rclcpp_action::ClientGoalHandle<ExecuteAction>;

  enum Status
  {
    EXECUTION_ERROR,
    AT_START_REQ_ERROR,
    OVER_ALL_REQ_ERROR,
    AT_END_REQ_ERROR,
    AT_START_EF_ERROR,
    AT_END_EF_ERROR,
    STARTING,
    EXECUTING,
    SUCCEEDED
  };

  explicit ActionExecutor(const std::string & action);
  ActionExecutor();

  void update();

  bool finished() {return finished_;}
  float getProgress() {return feedback_.progress;}
  Status getStatus() {return status_;}

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  rclcpp_action::Client<ExecuteAction>::SharedPtr execute_action_client_ptr_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_problem_sub_;

  ExecuteAction::Feedback feedback_;
  ExecuteAction::Result result_;
  bool finished_;
  Status status_;

  DurativeAction current_action_;

  rclcpp::Node::SharedPtr spin_node_;
  rclcpp::Node::SharedPtr aux_node_;

  void update_callback(const std_msgs::msg::Empty::SharedPtr msg);

  void feedback_callback(
    GoalHandleExecuteAction::SharedPtr,
    const std::shared_ptr<const ExecuteAction::Feedback> feedback);

  void result_callback(const GoalHandleExecuteAction::WrappedResult & result);

  bool update_current_action(const std::string & action_expr);

  bool executeAction();

  bool check(const PredicateTree & predicate_tree) {return check(predicate_tree.root_);}
  bool check(const std::shared_ptr<TreeNode> node) const;
  bool apply(const PredicateTree & predicate_tree) {return apply(predicate_tree.root_);}
  bool apply(const std::shared_ptr<TreeNode> node, bool negate = false) const;

  std::string get_name(const std::string & action_expr);
  std::vector<std::string> get_params(const std::string & action_expr);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
