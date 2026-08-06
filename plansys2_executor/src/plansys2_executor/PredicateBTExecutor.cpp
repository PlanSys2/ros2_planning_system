// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
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
// See the License for the specific la

#include "behaviortree_cpp/utils/shared_library.h"
#include "plansys2_executor/PredicateBTExecutor.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

PredicateBTExecutor::PredicateBTExecutor(BT::Blackboard::Ptr blackboard)
: blackboard_(blackboard)
{
  auto node = blackboard_->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  logger_ = node->get_logger();

  problem_client_ =
    blackboard_->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
  predicate_bt_map_ =
    blackboard_->get<std::unordered_map<std::string, std::string>>("predicates_bt_xml");
  predicate_plugin_list_ = blackboard_->get<std::vector<std::string>>("predicate_plugins");

  // Register the plugins for the behavior tree factory
  BT::SharedLibrary loader;
  for (auto plugin : predicate_plugin_list_) {
    factory_.registerFromPlugin(loader.getOSName(plugin));
  }
}

bool PredicateBTExecutor::check_predicate(const plansys2::Predicate & predicate)
{
  // Check if a specific behavior tree is defined for this predicate
  // If so, check if the predicate exists symbolically and then execute the BT
  // If not, just check if the predicate exists in the problem client
  if (auto it = predicate_bt_map_.find(predicate.name); it != predicate_bt_map_.end()) {
    RCLCPP_DEBUG(
      logger_,
      "Executing behavior tree for predicate: %s", parser::pddl::toString(predicate).c_str());
    set_arguments_in_blackboard(predicate);
    return problem_client_->existPredicate(predicate) && execute_bt(it->second);
  } else {
    RCLCPP_DEBUG(
      logger_,
      "No specific behavior tree for predicate '%s', using default check.",
        parser::pddl::toString(predicate).c_str());
    return problem_client_->existPredicate(predicate);
  }
}

void PredicateBTExecutor::set_arguments_in_blackboard(const plansys2::Predicate & predicate)
{
  for (int i = 0; i < predicate.parameters.size(); ++i) {
    auto arg = predicate.parameters[i].name;
    RCLCPP_DEBUG_STREAM(logger_, "Setting arg" << i << " [" << arg << "]");
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, arg);
  }
}

bool PredicateBTExecutor::execute_bt(const std::string & bt_xml_path)
{
  auto tree = factory_.createTreeFromFile(bt_xml_path, blackboard_);
  BT::NodeStatus status = tree.rootNode()->executeTick();
  return status == BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
