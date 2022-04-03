// Copyright 2022 Intelligent Robotics Lab
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

#ifndef BEHAVIOR_TREE__FAILURENODES_HPP_
#define BEHAVIOR_TREE__FAILURENODES_HPP_

#include <string>
#include <memory>

#include "test_msgs/action/fibonacci.hpp"

#include "plansys2_bt_actions/BTActionNode.hpp"

namespace plansys2_bt_tests
{

class OnTickFail : public plansys2::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  using Fibonacci = test_msgs::action::Fibonacci;
  explicit OnTickFail(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : plansys2::BtActionNode<Fibonacci>(xml_tag_name,
      action_name,
      conf),
    on_tick_run(false)
  {
  }

  BT::NodeStatus on_tick() override
  {
    on_tick_run = true;
    return BT::NodeStatus::FAILURE;
  }

  bool on_tick_run;
};


class OnFeedbackFail : public plansys2::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  using Fibonacci = test_msgs::action::Fibonacci;
  explicit OnFeedbackFail(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : plansys2::BtActionNode<Fibonacci>(xml_tag_name,
      action_name,
      conf),
    on_feedback_run(false)
  {
  }

  BT::NodeStatus on_tick() override
  {
    if (return_failure) {
      return BT::NodeStatus::FAILURE;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void on_feedback(const std::shared_ptr<const Fibonacci::Feedback>) override
  {
    on_feedback_run = true;
    return_failure = true;
  }

  bool on_feedback_run;
  bool return_failure {false};
};

}   // namespace plansys2_bt_tests

#endif  // BEHAVIOR_TREE__FAILURENODES_HPP_
