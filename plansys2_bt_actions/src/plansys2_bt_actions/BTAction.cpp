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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "plansys2_bt_actions/BTAction.hpp"

namespace plansys2
{

BTAction::BTAction(
  const std::string & action,
  const std::string & bt_xml_file,
  const std::vector<std::string> & plugin_list,
  const std::chrono::nanoseconds & rate)
: ActionExecutorClient(action, rate),
  action_(action),
  bt_xml_file_(bt_xml_file),
  plugin_list_(plugin_list)
{
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  for (auto plugin : plugin_list_) {
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", rclcpp::Node::make_shared(get_name()));
  tree_ = factory.createTreeFromFile(bt_xml_file_, blackboard_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  for (int i = 0; i < get_arguments().size(); i++) {
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, get_arguments()[i]);
  }

  return ActionExecutorClient::on_activate(previous_state);
}

void
BTAction::do_work()
{
  auto result = tree_.rootNode()->executeTick();

  switch (result) {
    case BT::NodeStatus::SUCCESS:
      finish(true, 1.0, "Action completed");
      break;
    case BT::NodeStatus::RUNNING:
      send_feedback(0.0, "Action running");
      break;
    case BT::NodeStatus::FAILURE:
      finish(false, 1.0, "Action failed");
      break;
  }
}

}  // namespace plansys2
