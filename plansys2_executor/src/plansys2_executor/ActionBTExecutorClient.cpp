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

#include "plansys2_executor/ActionBTExecutorClient.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>

#include "behaviortree_cpp_v3/basic_types.h"

namespace plansys2
{

ActionBTExecutorClient::ActionBTExecutorClient(
  const std::string & action,
  const std::string & bt_xml_file,
  float rate)
: ActionExecutorClient(action, rate),
  bt_xml_file_(bt_xml_file),
  factory_()
{
  blackboard_ = BT::Blackboard::create();
}

void
ActionBTExecutorClient::atStart()
{
  finished_ = false;

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_file_);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_file_.c_str());

    // ToDo (fmrico): We should manage errors
    finished_ = true;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_file_.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string.c_str());

  // Create the Behavior Tree from the XML input
  tree_ = factory_.createTreeFromText(xml_string, blackboard_);
}

void
ActionBTExecutorClient::actionStep()
{
  auto result = tree_.root_node->executeTick();

  switch (result) {
    case BT::NodeStatus::SUCCESS:
      getFeedback()->progress = 100.0;
      finished_ = true;
      break;
    case BT::NodeStatus::RUNNING:
      finished_ = false;
      break;
    case BT::NodeStatus::FAILURE:
      finished_ = true;
      break;
  }
}

bool
ActionBTExecutorClient::isFinished()
{
  return finished_;
}

}  // namespace plansys2
