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
#include <vector>

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"

#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/add_problem_instance.hpp"
#include "plansys2_msgs/srv/add_problem_predicate.hpp"
#include "plansys2_msgs/srv/add_problem_function.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_problem_predicate_details.hpp"
#include "plansys2_msgs/srv/get_problem_predicates.hpp"
#include "plansys2_msgs/srv/get_problem_function_details.hpp"
#include "plansys2_msgs/srv/get_problem_functions.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_msgs/srv/remove_problem_instance.hpp"
#include "plansys2_msgs/srv/remove_problem_predicate.hpp"
#include "plansys2_msgs/srv/remove_problem_function.hpp"
#include "plansys2_msgs/srv/exist_problem_predicate.hpp"
#include "plansys2_msgs/srv/exist_problem_function.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

class ProblemExpertClient : public ProblemExpertInterface
{
public:
  explicit ProblemExpertClient(rclcpp::Node::SharedPtr provided_node);

  std::vector<Instance> getInstances();
  bool addInstance(const Instance & instance);
  bool removeInstance(const std::string & name);
  boost::optional<Instance> getInstance(const std::string & name);

  std::vector<Predicate> getPredicates();
  bool addPredicate(const Predicate & predicate);
  bool removePredicate(const Predicate & predicate);
  bool existPredicate(const Predicate & predicate);

  std::vector<Function> getFunctions();
  bool addFunction(const Function & function);
  bool removeFunction(const Function & function);
  bool existFunction(const Function & function);

  Goal getGoal();
  bool setGoal(const Goal & goal);
  bool clearGoal();

  std::string getProblem();

private:
  rclcpp::Client<plansys2_msgs::srv::AddProblemGoal>::SharedPtr
    add_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::AddProblemInstance>::SharedPtr
    add_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AddProblemPredicate>::SharedPtr
    add_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AddProblemFunction>::SharedPtr
    add_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemGoal>::SharedPtr
    get_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
    get_problem_instance_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
    get_problem_instances_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemPredicateDetails>::SharedPtr
    get_problem_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemPredicates>::SharedPtr
    get_problem_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemFunctionDetails>::SharedPtr
    get_problem_function_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemFunctions>::SharedPtr
    get_problem_functions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblem>::SharedPtr
    get_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr
    remove_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemInstance>::SharedPtr
    remove_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemPredicate>::SharedPtr
    remove_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemFunction>::SharedPtr
    remove_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistProblemPredicate>::SharedPtr
    exist_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistProblemFunction>::SharedPtr
    exist_problem_function_client_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
