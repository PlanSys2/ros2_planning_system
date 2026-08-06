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
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_EXECUTOR__PREDICATEBTEXECUTOR_HPP_
#define PLANSYS2_EXECUTOR__PREDICATEBTEXECUTOR_HPP_

#include <string>
#include <unordered_map>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "plansys2_core/Types.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2
{

/**
 * @class plansys2::PredicateBTExecutor
 * @brief Class to execute behavior trees for checking predicates.
 */
class PredicateBTExecutor
{
public:
  /**
   * @brief Constructor for PredicateBTExecutor.
   *
   * @param blackboard The blackboard to be used by the behavior trees.
   */
  explicit PredicateBTExecutor(BT::Blackboard::Ptr blackboard);

  /**
   * @brief Execute a predicate check using the specified behavior tree.
   *
   * @param predicate The predicate to check.
   * @return bool Whether the predicate check was successful.
   */
  bool check_predicate(const plansys2::Predicate & predicate);

private:
  /**
   * @brief Execute the behavior tree for the given predicate.
   *
   * @param bt_xml_path The path to the behavior tree XML file.
   * @return true If the behavior tree execution was successful, false otherwise.
   */
  bool execute_bt(const std::string & bt_xml_path);

  /**
   * @brief Set the predicate arguments in the blackboard.
   *
   * @param predicate The predicate whose arguments are to be set.
   */
  void set_arguments_in_blackboard(const plansys2::Predicate & predicate);

  rclcpp::Logger logger_{rclcpp::get_logger("PredicateBTExecutor")};
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  // Plugin list for predicates
  std::vector<std::string> predicate_plugin_list_;
  // Map of predicate names to their corresponding behavior tree XML paths
  std::unordered_map<std::string, std::string> predicate_bt_map_;
  // Factory for creating behavior trees and blackboard
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

} // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__PREDICATEBTEXECUTOR_HPP_
