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

#include <string>
#include <iostream>

#include "Move.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace plansys2_bt_tests
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<test_msgs::action::Fibonacci>(xml_tag_name, action_name, conf)
{
}

void
Move::on_tick()
{
  geometry_msgs::msg::Pose2D goal;
  getInput<geometry_msgs::msg::Pose2D>("goal", goal);

  int order_to = static_cast<int>(goal.x);
  goal_.order = order_to;

  setOutput("goal_reached", 0);
}

BT::NodeStatus
Move::on_success()
{
  setOutput("goal_reached", result_.result->sequence[0]);

  return BT::NodeStatus::SUCCESS;
}


}  // namespace plansys2_bt_tests

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys2_bt_tests::Move>(
        name, "move", config);
    };

  factory.registerBuilder<plansys2_bt_tests::Move>(
    "Move", builder);
}