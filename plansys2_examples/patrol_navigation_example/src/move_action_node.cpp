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

#include <math.h>

#include <memory>
#include <string>
#include <map>

#include "plansys2_msgs/action/execute_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move")
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "/map";
    wp.pose.position.x = 0.0;
    wp.pose.position.y = -2.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    waypoints_["wp1"] = wp;

    wp.pose.position.x = 1.8;
    wp.pose.position.y = 0.0;
    waypoints_["wp2"] = wp;

    wp.pose.position.x = 0.0;
    wp.pose.position.y = 2.0;
    waypoints_["wp3"] = wp;

    wp.pose.position.x = -0.5;
    wp.pose.position.y = -0.5;
    waypoints_["wp4"] = wp;

    wp.pose.position.x = -2.0;
    wp.pose.position.y = -0.4;
    waypoints_["wp_control"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  void onActivate()
  {
    ActionExecutorClient::onActivate();
    getFeedback()->progress = 0.0;

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(),
        "NavigateToPose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate = getArguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {getFeedback()->progress = 100.0;};

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    navigation_goal_handle_ = future_navigation_goal_handle_.get();
    if (!navigation_goal_handle_) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    }
  }

private:
  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt((pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
             (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void actionStep()
  {
    // Nothing to do. Action is send and check is done in isFinished
    auto status = navigation_goal_handle_->get_status();

    // Check if the goal is still executing
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
      RCLCPP_DEBUG(get_logger(), "Executing move action");
    } else {
      RCLCPP_WARN(get_logger(), "Error Executing");
    }

    double dist_to_goal = getDistance(goal_pos_.pose, current_pos_);

    getFeedback()->progress = 100.0 * (1.0 - (dist_to_goal / dist_to_move));
  }

  bool isFinished()
  {
    if (getFeedback()->progress >= 100.0) {
      // Check result of navigation
      return true;
    } else {
      return false;
    }
  }

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
