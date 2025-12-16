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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_core/Types.hpp"

#include "std_msgs/msg/empty.hpp"

#include "plansys2_msgs/msg/problem.hpp"
#include "plansys2_msgs/srv/add_problem.hpp"
#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/affect_node.hpp"
#include "plansys2_msgs/srv/affect_param.hpp"
#include "plansys2_msgs/srv/exist_node.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"
#include "plansys2_msgs/srv/is_problem_goal_satisfied.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_msgs/srv/clear_problem_knowledge.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

/**
 * @class plansys2::ProblemExpertClient
 * @brief Client implementation for accessing the ProblemExpert node services.
 *
 * This class provides a client interface to interact with the ProblemExpert node,
 * implementing the ProblemExpertInterface. It allows managing PDDL problem elements
 * such as instances, predicates, functions, and goals through ROS 2 services.
 */
class ProblemExpertClient : public ProblemExpertInterface
{
public:
  /**
   * @brief Constructor for the ProblemExpertClient.
   *
   * Initializes a ROS node and creates all the service clients needed to
   * communicate with the ProblemExpert node.
   */
  ProblemExpertClient();

  /**
   * @brief Get all instances in the problem.
   *
   * @return std::vector<plansys2::Instance> Vector containing all instances in the problem.
   */
  std::vector<plansys2::Instance> getInstances();

  /**
   * @brief Add a new instance to the problem.
   *
   * @param[in] instance The instance to be added.
   * @return true if the instance was successfully added, false otherwise.
   */
  bool addInstance(const plansys2::Instance & instance);

  /**
   * @brief Remove an instance from the problem.
   *
   * @param[in] instance The instance to be removed.
   * @return true if the instance was successfully removed, false otherwise.
   */
  bool removeInstance(const plansys2::Instance & instance);

  /**
   * @brief Get a specific instance by name.
   *
   * @param[in] name The name of the instance to retrieve.
   * @return std::optional<plansys2::Instance> The instance if found, empty otherwise.
   */
  std::optional<plansys2::Instance> getInstance(const std::string & name);

  /**
   * @brief Get all predicates in the problem.
   *
   * @return std::vector<plansys2::Predicate> Vector containing all predicates in the problem.
   */
  std::vector<plansys2::Predicate> getPredicates();

  /**
   * @brief Add a new predicate to the problem.
   *
   * @param[in] predicate The predicate to be added.
   * @return true if the predicate was successfully added, false otherwise.
   */
  bool addPredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Remove a predicate from the problem.
   *
   * @param[in] predicate The predicate to be removed.
   * @return true if the predicate was successfully removed, false otherwise.
   */
  bool removePredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Check if a predicate exists in the problem.
   *
   * @param[in] predicate The predicate to check.
   * @return true if the predicate exists, false otherwise.
   */
  bool existPredicate(const plansys2::Predicate & predicate);

  /**
   * @brief Get a specific predicate by its expression.
   *
   * @param[in] expr The expression of the predicate to retrieve.
   * @return std::optional<plansys2::Predicate> The predicate if found, empty otherwise.
   */
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  /**
   * @brief Get all functions in the problem.
   *
   * @return std::vector<plansys2::Function> Vector containing all functions in the problem.
   */
  std::vector<plansys2::Function> getFunctions();

  /**
   * @brief Add a new function to the problem.
   *
   * @param[in] function The function to be added.
   * @return true if the function was successfully added, false otherwise.
   */
  bool addFunction(const plansys2::Function & function);

  /**
   * @brief Remove a function from the problem.
   *
   * @param[in] function The function to be removed.
   * @return true if the function was successfully removed, false otherwise.
   */
  bool removeFunction(const plansys2::Function & function);

  /**
   * @brief Check if a function exists in the problem.
   *
   * @param[in] function The function to check.
   * @return true if the function exists, false otherwise.
   */
  bool existFunction(const plansys2::Function & function);

  /**
   * @brief Update the value of an existing function.
   *
   * @param[in] function The function with the updated value.
   * @return true if the function was successfully updated, false otherwise.
   */
  bool updateFunction(const plansys2::Function & function);

  /**
   * @brief Get a specific function by its expression.
   *
   * @param[in] expr The expression of the function to retrieve .
   * @return std::optional<plansys2::Function> The function if found, empty otherwise.
   */
  std::optional<plansys2::Function> getFunction(const std::string & function);

  /**
   * @brief Get the current goal of the problem.
   *
   * @return plansys2::Goal The current goal.
   */
  plansys2::Goal getGoal();

  /**
   * @brief Set a new goal for the problem.
   *
   * @param[in] goal The goal to be set.
   * @return true if the goal was successfully set, false otherwise.
   */
  bool setGoal(const plansys2::Goal & goal);

  /**
   * @brief Check if a goal is satisfied in the current state.
   *
   * @param[in] goal The goal to check for satisfaction.
   * @return true if the goal is satisfied, false otherwise.
   */
  bool isGoalSatisfied(const plansys2::Goal & goal);

  /**
   * @brief Clear the current goal from the problem.
   *
   * @return true if the goal was successfully cleared, false otherwise.
   */
  bool clearGoal();

  /**
   * @brief Clear all knowledge (instances, predicates, functions, and goal) from the problem.
   *
   * @return true if the knowledge was successfully cleared, false otherwise.
   */
  bool clearKnowledge();

  /**
   * @brief Get the complete PDDL problem as a string.
   *
   * @return std::string The complete PDDL problem string.
   */
  std::string getProblem();

  /**
   * @brief Get the complete PDDL problem as a string, with optional caching.
   *
   * @param[in] use_cache If true, returns cached problem if available.
   * @return std::string The complete PDDL problem string.
   */
  std::string getProblem(bool use_cache);

  /**
   * @brief Get the complete PDDL problem as a string, with timestamp.
   *
   * @param[in] use_cache If true, returns cached problem if available.
   * @return std::tuple<std::string, rclcpp::Time> A tuple containing the PDDL problem string and its timestamp.
   */
  std::tuple<std::string, rclcpp::Time> getProblemWithTimestamp(bool use_cache = false);

  /**
   * @brief Add a complete PDDL problem from a string.
   *
   * @param[in] problem_str The PDDL problem string to parse and add.
   * @return true if the problem was successfully added, false otherwise.
   */
  bool addProblem(const std::string & problem_str);

  /**
   * @brief Get the timestamp of the last problem update.
   *
   * @return rclcpp::Time The timestamp when the problem was last updated.
   */
  rclcpp::Time getUpdateTime() const {return update_time_;}

  std::string cached_problem_;
  rclcpp::Time problem_ts_;

private:
  rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr
    add_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::AddProblemGoal>::SharedPtr
    add_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectParam>::SharedPtr
    add_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemGoal>::SharedPtr
    get_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
    get_problem_instance_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
    get_problem_instances_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_function_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_functions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblem>::SharedPtr
    get_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr
    remove_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::ClearProblemKnowledge>::SharedPtr
    clear_problem_knowledge_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectParam>::SharedPtr
    remove_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    update_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::IsProblemGoalSatisfied>::SharedPtr
    is_problem_goal_satisfied_client_;
  rclcpp::Subscription<plansys2_msgs::msg::Problem>::SharedPtr problem_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_problem_sub_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time update_time_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
