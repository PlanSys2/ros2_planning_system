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

#include <stdio.h>
#include <readline/readline.h>

#include <readline/history.h>

#include <regex>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <sstream>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"

#include "plansys2_msgs/msg/action_performer_status.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

#include "plansys2_terminal/Terminal.hpp"


namespace plansys2_terminal
{

std::vector<std::string> tokenize(const std::string & text)
{
  if (text.empty()) {
    return {};
  }

  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(" ", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}

void pop_front(std::vector<std::string> & tokens)
{
  if (!tokens.empty()) {
    tokens.erase(tokens.begin(), tokens.begin() + 1);
  }
}

// LCOV_EXCL_START
char * completion_generator(const char * text, int state)
{
  // This function is called with state=0 the first time; subsequent calls are
  // with a nonzero state. state=0 can be used to perform one-time
  // initialization for this completion session.
  static std::vector<std::string> matches;
  static size_t match_index = 0;

  std::vector<std::string> vocabulary{"get", "set", "remove", "run", "check"};
  std::vector<std::string> vocabulary_check{"actors"};
  std::vector<std::string> vocabulary_set{"instance", "predicate", "function", "goal"};
  std::vector<std::string> vocabulary_get{"model", "problem", "domain", "plan"};
  std::vector<std::string> vocabulary_remove{"instance", "predicate", "function", "goal"};
  std::vector<std::string> vocabulary_get_problem{"instances", "predicates", "functions", "goal"};
  std::vector<std::string> vocabulary_get_model{"types", "predicates", "functions", "actions",
    "predicate", "function", "action"};

  if (state == 0) {
    // During initialization, compute the actual matches for 'text' and keep
    // them in a static vector.
    matches.clear();
    match_index = 0;

    // Collect a vector of matches: vocabulary words that begin with text.
    std::string textstr = std::string(text);

    auto current_text = tokenize(rl_line_buffer);
    std::vector<std::string> * current_vocabulary = nullptr;

    if (current_text.size() <= 1) {
      current_vocabulary = &vocabulary;
    } else {
      if (current_text.size() == 2) {
        if (current_text[0] == "set") {
          current_vocabulary = &vocabulary_set;
        } else if (current_text[0] == "get") {
          current_vocabulary = &vocabulary_get;
        } else if (current_text[0] == "remove") {
          current_vocabulary = &vocabulary_remove;
        } else if (current_text[0] == "check") {
          current_vocabulary = &vocabulary_check;
        }
      } else if (current_text.size() == 3) {
        if (current_text[0] == "get" && current_text[1] == "problem") {
          current_vocabulary = &vocabulary_get_problem;
        } else if (current_text[0] == "get" && current_text[1] == "model") {
          current_vocabulary = &vocabulary_get_model;
        }
      }
    }

    if (current_vocabulary == nullptr) {
      return nullptr;
    }

    for (auto word : *current_vocabulary) {
      if (word.size() >= textstr.size() &&
        word.compare(0, textstr.size(), textstr) == 0)
      {
        matches.push_back(word);
      }
    }
  }

  if (match_index >= matches.size()) {
    // We return nullptr to notify the caller no more matches are available.
    return nullptr;
  } else {
    // Return a malloc'd char* for the match. The caller frees it.
    return strdup(matches[match_index++].c_str());
  }
}

char ** completer(const char * text, int start, int end)
{
  // Don't do filename completion even if our generator finds no matches.
  rl_attempted_completion_over = 1;

  // Note: returning nullptr here will make readline use the default filename
  // completer.

  return rl_completion_matches(text, completion_generator);
}


Terminal::Terminal()
: rclcpp::Node("terminal")
{
}

void
Terminal::init()
{
  std::shared_ptr<rclcpp::Node> terminal_node = shared_from_this();
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(terminal_node);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(terminal_node);
  planner_client_ = std::make_shared<plansys2::PlannerClient>(terminal_node);
  executor_client_ = std::make_shared<plansys2::ExecutorClient>(terminal_node);
}

void
Terminal::run_console()
{
  init();

  std::string line;
  bool success = true;

  std::cout << "ROS2 Planning System console. Type \"quit\" to finish" << std::endl;

  rl_attempted_completion_function = completer;

  bool finish = false;
  while (!finish) {
    char * line = readline("> ");

    if (line == NULL || (strcmp(line, "quit") == 0)) {
      finish = true;
    }

    if (strlen(line) > 0) {
      add_history(line);

      std::string line_str(line);
      free(line);

      if (!finish) {
        clean_command(line_str);

        std::ostringstream os;
        process_command(line_str, os);
        std::cout << os.str();
      }
    }
  }

  std::cout << "Finishing..." << std::endl;
}
// LCOV_EXCL_STOP

void
Terminal::clean_command(std::string & command)
{
  // remove continuous spaces
  size_t pos;
  while ((pos = command.find("  ")) != command.npos) {
    command.erase(pos, 1);
  }

  // remove from spaces
  while (*command.begin() == ' ') {
    command.erase(0, 1);
  }

  // remove trailing spaces
  while (command[command.size() - 1] == ' ') {
    command.pop_back();
  }
}

void
Terminal::process_get_model_predicate(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    auto predicates = domain_client_->getPredicate(command[0]);
    if (predicates) {
      os << "Parameters: " << predicates.value().parameters.size() << std::endl;
      for (size_t i = 0; i < predicates.value().parameters.size(); i++) {
        os << "\t" << predicates.value().parameters[i].type << " - " <<
          predicates.value().parameters[i].name << std::endl;
      }
    } else {
      os << "Error when looking for params of " << command[0] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget model predicate [predicate_name]" << std::endl;
  }
}

void
Terminal::process_get_model_function(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    auto functions = domain_client_->getFunction(command[0]);
    if (functions) {
      os << "Parameters: " << functions.value().parameters.size() << std::endl;
      for (size_t i = 0; i < functions.value().parameters.size(); i++) {
        os << "\t" << functions.value().parameters[i].type << " - " <<
          functions.value().parameters[i].name << std::endl;
      }
    } else {
      os << "Error when looking for params of " << command[0] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget model function [function_name]" << std::endl;
  }
}

void
Terminal::process_get_model_action(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    auto action = domain_client_->getAction(command[0]);
    auto durative_action = domain_client_->getDurativeAction(command[0]);
    if (action) {
      os << "Type: action" << std::endl;
      os << "Parameters: " << action.value().parameters.size() << std::endl;
      for (size_t i = 0; i < action.value().parameters.size(); i++) {
        os << "\t" << action.value().parameters[i].type << " - " <<
          action.value().parameters[i].name << std::endl;
      }
      os << "Preconditions: " << action.value().preconditions.toString() << std::endl;
      os << "Effects: " << action.value().effects.toString() << std::endl;
    } else if (durative_action) {
      os << "Type: durative-action" << std::endl;
      os << "Parameters: " << durative_action.value().parameters.size() << std::endl;
      for (size_t i = 0; i < durative_action.value().parameters.size(); i++) {
        os << "\t" << durative_action.value().parameters[i].name << " - " <<
          durative_action.value().parameters[i].type << std::endl;
      }
      os << "AtStart requirements: " <<
        durative_action.value().at_start_requirements.toString() << std::endl;
      os << "OverAll requirements: " <<
        durative_action.value().over_all_requirements.toString() << std::endl;
      os << "AtEnd requirements: " <<
        durative_action.value().at_end_requirements.toString() << std::endl;
      os << "AtStart effect: " <<
        durative_action.value().at_start_effects.toString() << std::endl;
      os << "AtEnd effect: " <<
        durative_action.value().at_end_effects.toString() << std::endl;
    } else {
      os << "Error when looking for params of " << command[0] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget model action [action_name]" << std::endl;
  }
}

void
Terminal::process_get_model(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "types") {
      auto types = domain_client_->getTypes();

      os << "Types: " << types.size() << std::endl;
      for (const auto & type : types) {
        os << "\t" << type << std::endl;
      }
    } else if (command[0] == "predicates") {
      auto predicates = domain_client_->getPredicates();

      os << "Predicates: " << predicates.size() << std::endl;
      for (const auto & predicate : predicates) {
        os << "\t" << predicate << std::endl;
      }
    } else if (command[0] == "functions") {
      auto functions = domain_client_->getFunctions();

      os << "Functions: " << functions.size() << std::endl;
      for (const auto & function : functions) {
        os << "\t" << function << std::endl;
      }
    } else if (command[0] == "actions") {
      auto actions = domain_client_->getActions();
      auto durative_actions = domain_client_->getDurativeActions();

      os << "Actions: " << actions.size() << std::endl;
      for (const auto & action : actions) {
        os << "\t" << action << " (action)" << std::endl;
      }
      for (const auto & durative_action : durative_actions) {
        os << "\t" << durative_action << " (durative action)" << std::endl;
      }
    } else if (command[0] == "predicate") {
      pop_front(command);
      process_get_model_predicate(command, os);
    } else if (command[0] == "function") {
      pop_front(command);
      process_get_model_function(command, os);
    } else if (command[0] == "action") {
      pop_front(command);
      process_get_model_action(command, os);
    } else {
      os <<
        "\tUsage: \n\t\tget model [types|predicates|functions|actions|predicate|function|action]..."
         <<
        std::endl;
    }
  } else {
    os <<
      "\tUsage: \n\t\tget model [types|predicates|functions|actions|predicate|function|action]..."
       <<
      std::endl;
  }
}

void
Terminal::process_get_problem(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "instances") {
      auto instances = problem_client_->getInstances();

      os << "Instances: " << instances.size() << std::endl;
      for (const auto & instance : instances) {
        os << "\t" << instance.name << "\t" << instance.type << std::endl;
      }
    } else if (command[0] == "predicates") {
      auto predicates = problem_client_->getPredicates();

      os << "Predicates: " << predicates.size() << std::endl;
      for (const auto & predicate : predicates) {
        os << predicate.toString() << std::endl;
      }
    } else if (command[0] == "functions") {
      auto functions = problem_client_->getFunctions();

      os << "Functions: " << functions.size() << std::endl;
      for (const auto & function : functions) {
        os << function.toString() << std::endl;
      }
    } else if (command[0] == "goal") {
      auto goal = problem_client_->getGoal();
      os << "Goal: " << goal.toString() << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget problem [instances|predicates|functions|goal]..." <<
      std::endl;
  }
}

void
Terminal::process_get(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "model") {
      pop_front(command);
      process_get_model(command, os);
    } else if (command[0] == "problem") {
      pop_front(command);
      process_get_problem(command, os);
    } else if (command[0] == "domain") {
      os << "domain: \n" << domain_client_->getDomain() << std::endl;
    } else if (command[0] == "plan") {
      auto plan = planner_client_->getPlan(
        domain_client_->getDomain(),
        problem_client_->getProblem());

      if (plan) {
        os << "plan: " << std::endl;
        for (const auto & action : plan.value()) {
          os << action.time << "\t" << action.action << "\t" <<
            action.duration << std::endl;
        }
      } else {
        os << "No se ha encontrado plan" << std::endl;
      }
    } else {
      os << " get ---> " << command[0] << std::endl;
      os << "\tUsage: \n\t\tget [model|problem|domain|plan]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget [model|problem|domain|plan]..." <<
      std::endl;
  }
}

void
Terminal::process_set_instance(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 2) {
    if (!problem_client_->addInstance(parser::pddl::tree::Instance{command[0], command[1]})) {
      os << "Could not add the instance [" << command[0] << "]" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tset instance [name] [type]" <<
      std::endl;
  }
}

void
Terminal::process_set_predicate(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    parser::pddl::tree::Predicate predicate;
    predicate.name = command[0];

    if (predicate.name.front() != '(') {
      os << "\tUsage: \n\t\tset predicate (predicate)" <<
        std::endl;
      return;
    }

    predicate.name.erase(0, 1);  // Remove first )

    pop_front(command);
    while (!command.empty()) {
      parser::pddl::tree::Param param {command[0], ""};
      predicate.parameters.push_back(param);
      pop_front(command);
    }

    if (predicate.parameters.back().name.back() != ')') {
      os << "\tUsage: \n\t\tset predicate (predicate)" <<
        std::endl;
      return;
    }

    predicate.parameters.back().name.pop_back();  // Remove last (

    if (!problem_client_->addPredicate(predicate)) {
      os << "Could not add the predicate [" << predicate.toString() << "]" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tset predicate [predicate]" <<
      std::endl;
  }
}

void
Terminal::process_set_function(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    parser::pddl::tree::Function function;

    std::string total_expr;
    for (const auto & token : command) {
      total_expr += " " + token;
    }

    function.fromString(total_expr);

    if (!problem_client_->addFunction(function)) {
      os <<
        "Could not add the function [" <<
        function.toString() << "]" << std::endl;
    } else {
      os << "done" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tset function [function]" <<
      std::endl;
  }
}

void
Terminal::process_set_goal(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    std::string total_expr;
    for (const auto & token : command) {
      total_expr += " " + token;
    }

    parser::pddl::tree::Goal goal(total_expr);

    if (goal.root_ != nullptr) {
      if (!problem_client_->setGoal(goal)) {
        os << "Could not set the goal [" << goal.toString() << "]" << std::endl;
      }
    } else {
      os << "\tUsage: \n\t\tset goal [goal]" <<
        std::endl;
    }
  } else {
    os << "Not valid goal" << std::endl;
  }
}

void
Terminal::process_set(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "instance") {
      pop_front(command);
      process_set_instance(command, os);
    } else if (command[0] == "predicate") {
      pop_front(command);
      process_set_predicate(command, os);
    } else if (command[0] == "function") {
      pop_front(command);
      process_set_function(command, os);
    } else if (command[0] == "goal") {
      pop_front(command);
      process_set_goal(command, os);
    } else {
      os << "\tUsage: \n\t\tset [instance|predicate|function|goal]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tset [instance|predicate|function|goal]..." <<
      std::endl;
  }
}

void
Terminal::process_remove_instance(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    if (!problem_client_->removeInstance(command[0])) {
      os << "Could not remove the instance [" << command[0] << "]" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove instance [name]" <<
      std::endl;
  }
}

void
Terminal::process_remove_predicate(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    parser::pddl::tree::Predicate predicate;
    predicate.name = command[0];

    if (predicate.name.front() != '(') {
      os << "\tUsage: \n\t\tremove predicate (predicate)" <<
        std::endl;
    }

    predicate.name.erase(0, 1);  // Remove first )

    pop_front(command);
    while (!command.empty()) {
      parser::pddl::tree::Param param {command[0], ""};
      predicate.parameters.push_back(param);
      pop_front(command);
    }

    if (predicate.parameters.back().name.back() != ')') {
      os << "\tUsage: \n\t\tremove predicate (predicate)" <<
        std::endl;
      return;
    }

    predicate.parameters.back().name.pop_back();  // Remove last (

    if (!problem_client_->removePredicate(predicate)) {
      os << "Could not remove the predicate [" << predicate.toString() << "]" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\remove predicate [name] [instance1] [instance2] ...[instaceN]" <<
      std::endl;
  }
}

void
Terminal::process_remove_function(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    parser::pddl::tree::Function function;

    std::regex name_regexp("[a-zA-Z][a-zA-Z0-9_\\-]*");

    std::smatch match;
    std::string temp;

    for (const auto & token : command) {
      temp += token + " ";
    }

    if (std::regex_search(temp, match, name_regexp)) {
      function.name = match.str(0);
      temp = match.suffix().str();
    }

    while (std::regex_search(temp, match, name_regexp)) {
      function.parameters.push_back(parser::pddl::tree::Param{match.str(0), ""});
      temp = match.suffix().str();
    }

    if (!problem_client_->removeFunction(function)) {
      os << "Could not remove the function [" << function.toString() << "]" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\remove function [name] [instance1] [instance2] ...[instaceN]" <<
      std::endl;
  }
}

void
Terminal::process_remove(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "instance") {
      pop_front(command);
      process_remove_instance(command, os);
    } else if (command[0] == "predicate") {
      pop_front(command);
      process_remove_predicate(command, os);
    } else if (command[0] == "function") {
      pop_front(command);
      process_remove_function(command, os);
    } else if (command[0] == "goal", os) {
      problem_client_->clearGoal();
    } else {
      os << "\tUsage: \n\t\tremove [instance|predicate|function|goal]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove [instance|predicate|function|goal]..." <<
      std::endl;
  }
}

void
Terminal::execute_plan()
{
  rclcpp::Rate loop_rate(5);

  if (!executor_client_->start_plan_execution()) {
    std::cout << "Execution could not start " << std::endl;
    return;
  }

  while (rclcpp::ok() && executor_client_->execute_and_check_plan()) {
    auto feedback = executor_client_->getFeedBack();

    std::cout << "\r\e[K" << std::flush;
    for (const auto & action_status : feedback.action_execution_status) {
      if (action_status.status == plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED ||
        action_status.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
      {
        continue;
      }
      std::cout << "[(" << action_status.action;

      for (const auto & param : action_status.arguments) {
        std::cout << " " << param;
      }
      std::cout << ") ";

      switch (action_status.status) {
        case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
          std::cout << "waiting]";
          break;
        case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
          std::cout << action_status.completion * 100.0 << "%]";
          break;
        case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
          std::cout << "FAILED]";
          break;
        case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
          std::cout << "succeeded]";
          break;
      }
    }

    std::cout << std::flush;

    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }

  std::cout << std::endl;

  if (executor_client_->getResult().value().success) {
    std::cout << "Successful finished " << std::endl;
  } else {
    std::cout << "Finished with error: " << std::endl;
  }
}


void
Terminal::process_run(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 0) {
    execute_plan();
  }

  // ToDo(fmrico): We should be able to run directly an action, for example:
  // run (move leia entrance dinning)
}

void
Terminal::process_check_actors(std::vector<std::string> & command, std::ostringstream & os)
{
  std::map<std::string, plansys2_msgs::msg::ActionPerformerStatus> actors;

  auto status_callback =
    [this, &actors](const plansys2_msgs::msg::ActionPerformerStatus::SharedPtr msg) {
      actors[msg->node_name] = *msg;
    };

  auto status_sub = create_subscription<plansys2_msgs::msg::ActionPerformerStatus>(
    "/performers_status", rclcpp::QoS(100).reliable(), status_callback);

  auto start = now();
  while (rclcpp::ok() && (now() - start).seconds() < 2.0) {
    rclcpp::spin_some(shared_from_this());
  }

  std::list<std::string> keys;
  for (const auto & actor : actors) {
    keys.push_back(actor.first);
  }
  keys.sort();

  for (const auto & key : keys) {
    os << "\t[" << key << "] " << actors[key].action;

    switch (actors[key].state) {
      case plansys2_msgs::msg::ActionPerformerStatus::NOT_READY:
        os << "\tNOT READY" << std::endl;
        break;
      case plansys2_msgs::msg::ActionPerformerStatus::READY:
        os << "\tREADY" << std::endl;
        break;
      case plansys2_msgs::msg::ActionPerformerStatus::RUNNING:
        os << "\tRUNNING" << std::endl;
        break;
      case plansys2_msgs::msg::ActionPerformerStatus::FAILURE:
        os << "\tFAILURE" << std::endl;
        break;
    }
  }
}

void
Terminal::process_check(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.empty()) {
    return;
  }

  if (command[0] == "actors") {
    process_check_actors(command, os);
  }

  // ToDo(fmrico): We should be able to run directly an action, for example:
  // run (move leia entrance dinning)
}

void
Terminal::process_command(std::string & command, std::ostringstream & os)
{
  std::vector<std::string> tokens = tokenize(command);

  if (tokens.empty()) {
    return;
  }

  if (tokens[0] == "get") {
    pop_front(tokens);
    process_get(tokens, os);
  } else if (tokens[0] == "set") {
    pop_front(tokens);
    process_set(tokens, os);
  } else if (tokens[0] == "remove") {
    pop_front(tokens);
    process_remove(tokens, os);
  } else if (tokens[0] == "run") {
    pop_front(tokens);
    process_run(tokens, os);
  } else if (tokens[0] == "check") {
    pop_front(tokens);
    process_check(tokens, os);
  } else {
    os << "Command not found" << std::endl;
  }
}

}  // namespace plansys2_terminal
