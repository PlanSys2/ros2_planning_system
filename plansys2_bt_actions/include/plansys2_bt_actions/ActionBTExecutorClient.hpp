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

#ifndef PLANSYS2_BT_ACTIONS__ACTIONBTEXECUTORCLIENT_HPP_
#define PLANSYS2_BT_ACTIONS__ACTIONBTEXECUTORCLIENT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

class ActionBTExecutorClient : public plansys2::ActionExecutorClient
{
public:
  ActionBTExecutorClient(
    const std::string & node_name,
    const std::string & bt_xml_file,
    float rate = 5);

  BT::Blackboard::Ptr getBlackBoard() {return blackboard_;}

protected:
  virtual void atStart();
  virtual void atSuccess() {}
  virtual void actionStep();
  bool isFinished();

  BT::Blackboard::Ptr getBackboard() {return blackboard_;}

  BT::BehaviorTreeFactory factory_;

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::string bt_xml_file_;

  bool finished_;
};

}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__ACTIONBTEXECUTORCLIENT_HPP_
