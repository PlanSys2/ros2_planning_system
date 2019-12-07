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

#include <memory>

#include "plansys2_msgs/action/execute_action.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AskChargeAction : public rclcpp::Node
{
public:
  using ExecuteAction = plansys2_msgs::action::ExecuteAction;
  using GoalHandleExecuteAction = rclcpp_action::ServerGoalHandle<ExecuteAction>;

  AskChargeAction()
  : rclcpp::Node("ask_charge"), canceled_(false)
  {
    using namespace std::placeholders;

    this->execute_AskCharge_action_server_ = rclcpp_action::create_server<ExecuteAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "ask_charge",
      std::bind(&AskChargeAction::handle_goal, this, _1, _2),
      std::bind(&AskChargeAction::handle_cancel, this, _1),
      std::bind(&AskChargeAction::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ExecuteAction>::SharedPtr execute_AskCharge_action_server_;
  bool canceled_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received [%s] action request", goal->action.c_str());

    for (size_t i = 0; i < goal->arguments.size(); i++) {
      RCLCPP_INFO(this->get_logger(), " Argument %zu: [%s]", i, goal->arguments[i].c_str());
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel AskCharge action");

    canceled_ = true;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteAction> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&AskChargeAction::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleExecuteAction> goal_handle)
  {
    rclcpp::Rate loop_rate(5);
    auto feedback = std::make_shared<ExecuteAction::Feedback>();
    auto result = std::make_shared<ExecuteAction::Result>();

    feedback->progress = 0.0;
    while (rclcpp::ok() && !canceled_ && feedback->progress < 100) {
      std::cerr << " " << feedback->progress << "%" << std::endl;

      std::cout << "\r\e[K" << std::flush;
      std::cout << "Asking for charge ... [" << feedback->progress << "%]  " << std::flush;

      feedback->progress += 5.0;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    if (canceled_) {
      result->success = false;
      result->error_info = "Charging action cancelled";
      goal_handle->canceled(result);

    } else {
      result->success = true;
      result->error_info = "";
      goal_handle->succeed(result);
    }

    canceled_ = false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AskChargeAction>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
