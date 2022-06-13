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

#ifndef PLANSYS2_BT_ACTIONS__BTACTION_HPP_
#define PLANSYS2_BT_ACTIONS__BTACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

class BTAction : public plansys2::ActionExecutorClient
{
public:
  explicit BTAction(
    const std::string & action,
    const std::chrono::nanoseconds & rate);

  const std::string & getActionName() const {return action_;}
  const std::string & getBTFile() const {return bt_xml_file_;}

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void do_work();

  BT::BehaviorTreeFactory factory_;

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string action_;
  std::string bt_xml_file_;
  std::vector<std::string> plugin_list_;
  bool finished_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;
  std::unique_ptr<BT::FileLogger> bt_file_logger_;
  std::unique_ptr<BT::MinitraceLogger> bt_minitrace_logger_;
};

}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__BTACTION_HPP_
