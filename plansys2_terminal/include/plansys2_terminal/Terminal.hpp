// Copyright 2020 Intelligent Robotics Lab
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

#ifndef PLANSYS2_TERMINAL__TERMINAL_HPP_
#define PLANSYS2_TERMINAL__TERMINAL_HPP_

#include <vector>
#include <string>
#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"

namespace plansys2_terminal
{

std::vector<std::string> tokenize(const std::string & text);
void pop_front(std::vector<std::string> & tokens);
char * completion_generator(const char * text, int state);
char ** completer(const char * text, int start, int end);

std::optional<plansys2_msgs::msg::Plan>
parse_plan(const std::string planfile);

class Terminal : public rclcpp::Node
{
public:
  Terminal();

  virtual void run_console();

protected:
  virtual void init();

  virtual void add_problem();

  virtual void clean_command(std::string & command);

  virtual void process_get_model_predicate(
    std::vector<std::string> & command,
    std::ostringstream & os);
  virtual void process_get_model_function(
    std::vector<std::string> & command,
    std::ostringstream & os);
  virtual void process_get_model_action(
    std::vector<std::string> & command,
    std::ostringstream & os);
  virtual void process_get_model(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_get_problem(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_get(std::vector<std::string> & command, std::ostringstream & os);

  virtual void process_set_instance(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_set_predicate(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_set_function(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_set_goal(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_set(std::vector<std::string> & command, std::ostringstream & os);

  virtual void process_remove_instance(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_remove_predicate(
    std::vector<std::string> & command,
    std::ostringstream & os);
  virtual void process_remove_function(
    std::vector<std::string> & command,
    std::ostringstream & os);
  virtual void process_remove(std::vector<std::string> & command, std::ostringstream & os);

  virtual void execute_plan(const plansys2_msgs::msg::Plan & plan);
  virtual void execute_plan(int items = -1);
  virtual void execute_action(std::vector<std::string> & command);
  virtual void process_run(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_check(std::vector<std::string> & command, std::ostringstream & os);
  virtual void process_check_actors(std::vector<std::string> & command, std::ostringstream & os);

  // Returns false if the processed command is violating some
  // restriction, e.g. there are nested source commands
  virtual bool process_command(
    std::string & command, std::ostringstream & os,
    bool inside_source = false);

  virtual void process_source(std::vector<std::string> & command, std::ostringstream & os);

  virtual void process_help(std::vector<std::string> & command, std::ostringstream & os);

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::string problem_file_name_;
};

}  // namespace plansys2_terminal

#endif  // PLANSYS2_TERMINAL__TERMINAL_HPP_
