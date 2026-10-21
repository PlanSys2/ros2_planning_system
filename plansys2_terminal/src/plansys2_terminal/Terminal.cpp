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
#include <fstream>
#include <map>
#include <mutex>
#include <utility>
#include <algorithm>
#include <atomic>
#include <charconv>
#include <optional>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/action_performer_status.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

#include "plansys2_terminal/Terminal.hpp"


namespace plansys2_terminal
{

std::vector<std::string> tokenize(const std::string & text)
{
  // The first line, split on blanks: tabs and '\r' separate as spaces do
  std::vector<std::string> ret;
  std::string token;
  for (char c : text.substr(0, text.find('\n'))) {
    if (c == ' ' || c == '\t' || c == '\r') {
      if (!token.empty()) {
        ret.push_back(token);
        token.clear();
      }
    } else {
      token += c;
    }
  }
  if (!token.empty()) {
    ret.push_back(token);
  }
  return ret;
}

namespace
{
std::atomic<bool> interrupted{false};

std::string join(const std::vector<std::string> & tokens)
{
  std::string ret;
  for (const auto & token : tokens) {
    ret += (ret.empty() ? "" : " ") + token;
  }
  return ret;
}

// "(name arg1 ... argN)": one predicate, nothing before or after it
std::optional<plansys2::Predicate> parse_predicate(const std::vector<std::string> & tokens)
{
  const std::string text = join(tokens);
  if (text.size() < 3 || text.front() != '(' || text.back() != ')' ||
    std::count(text.begin(), text.end(), '(') != 1 ||
    std::count(text.begin(), text.end(), ')') != 1)
  {
    return {};
  }
  auto predicate = parser::pddl::fromStringPredicate(text);
  if (!parser::pddl::isValidName(predicate.name)) {
    return {};
  }
  return predicate;
}
}  // namespace

void interrupt()
{
  interrupted = true;
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

  std::vector<std::string> vocabulary{"get", "set", "remove", "run",
    "check", "source", "help", "?", "quit"};
  std::vector<std::string> vocabulary_check{"actors"};
  std::vector<std::string> vocabulary_set{
    "instance", "predicate", "function", "goal", "metric"};
  std::vector<std::string> vocabulary_get{"model", "problem", "domain", "plan"};
  std::vector<std::string> vocabulary_remove{
    "instance", "predicate", "function", "goal", "metric"};
  std::vector<std::string> vocabulary_run{"action", "num_actions", "plan-file"};
  std::vector<std::string> vocabulary_get_problem{
    "instances", "predicates", "functions", "goal", "metric"};
  std::vector<std::string> vocabulary_get_model{"types", "predicates", "functions", "actions",
    "predicate", "function", "action", "derived", "deriveds"};
  // The help is initialized with all possible commands in the vocabulary
  std::vector<std::string> vocabulary_help(vocabulary);

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
        } else if (current_text[0] == "run") {
          current_vocabulary = &vocabulary_run;
        } else if (current_text[0] == "check") {
          current_vocabulary = &vocabulary_check;
        } else if ((current_text[0] == "help") || (current_text[0] == "?")) {
          current_vocabulary = &vocabulary_help;
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
  (void)start;
  (void)end;
  // Don't do filename completion even if our generator finds no matches.
  rl_attempted_completion_over = 1;

  // Note: returning nullptr here will make readline use the default filename
  // completer.

  return rl_completion_matches(text, completion_generator);
}
// LCOV_EXCL_STOP

std::optional<plansys2_msgs::msg::Plan>
parse_plan(const std::string planfile)
{
  std::ifstream plan_file(planfile, std::ifstream::in);
  if (plan_file.fail()) {
    std::cerr << "Unable to open plan file \"" << planfile <<
      "\" for reading!" << std::endl;
    return {};
  }

  // "time: (action args) [duration]", as the planners write them; empty lines and
  // comments (";") are skipped, and the duration is optional
  plansys2_msgs::msg::Plan ret;
  std::string line;
  for (int number = 1; getline(plan_file, line); number++) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == ';') {
      continue;
    }
    const size_t colon = line.find(':');
    const size_t open = line.find('(');
    const size_t close = line.find(')', open == std::string::npos ? 0 : open);
    double time = 0.0;
    double duration = 0.0;
    bool valid = colon != std::string::npos && open != std::string::npos &&
      close != std::string::npos && colon < open &&
      parser::pddl::parseNumber(line.substr(0, colon), time);
    if (valid) {
      const std::string rest = line.substr(close + 1);
      const auto rest_first = rest.find_first_not_of(" \t\r");
      if (rest_first != std::string::npos) {
        const size_t bra = rest.find('[');
        const size_t ket = rest.find(']');
        valid = rest_first == bra && ket != std::string::npos && bra < ket &&
          rest.find_first_not_of(" \t\r", ket + 1) == std::string::npos &&
          parser::pddl::parseNumber(rest.substr(bra + 1, ket - bra - 1), duration);
      }
    }
    if (!valid) {
      std::cerr << "Wrong line " << number << " in plan file \"" << planfile << "\": " <<
        line << std::endl;
      return {};
    }
    plansys2_msgs::msg::PlanItem item;
    item.time = time;
    item.action = parser::pddl::getReducedString(line.substr(open, close - open + 1));
    item.duration = duration;
    ret.items.push_back(item);
  }

  std::cout << "The plan read from \"" << planfile << "\" is " << std::endl;
  for (const auto & plan_item : ret.items) {
    std::cout << plan_item.time << ":\t" << plan_item.action <<
      "\t[" << plan_item.duration << "]" << std::endl;
  }

  if (ret.items.empty()) {
    return {};
  }
  return ret;
}

Terminal::Terminal(const rclcpp::NodeOptions & options)
: rclcpp::Node("terminal", options)
{
  this->declare_parameter<std::string>("problem_file", "");
}

void
Terminal::init()
{
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>(*this);
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(*this);
  planner_client_ = std::make_shared<plansys2::PlannerClient>(*this);
  executor_client_ = std::make_shared<plansys2::ExecutorClient>(*this);

  add_problem();
}

void Terminal::add_problem()
{
  this->get_parameter<std::string>("problem_file", problem_file_name_);
  if (!problem_file_name_.empty()) {
    RCLCPP_INFO(
      this->get_logger(), "Adding problem file to problem_expert: %s",
      problem_file_name_.c_str());
    std::ifstream problem_ifs(problem_file_name_);
    std::string problem_str((std::istreambuf_iterator<char>(
        problem_ifs)), std::istreambuf_iterator<char>());

    if (!problem_client_->addProblem(problem_str)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add problem to problem_expert.");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "No problem file specified.");
  }
}


// LCOV_EXCL_START
namespace
{
// Ctrl-C at the prompt discards the line, as in a shell
int new_line_on_interrupt()
{
  if (interrupted.exchange(false)) {
    rl_replace_line("", 0);
    rl_crlf();
    rl_on_new_line();
    rl_redisplay();
  }
  return 0;
}
}  // namespace

void
Terminal::run_console()
{
  init();

  std::cout << "ROS2 Planning System console. Type \"quit\" or Ctrl-D to finish" << std::endl;

  rl_attempted_completion_function = completer;
  rl_signal_event_hook = new_line_on_interrupt;

  bool finish = false;
  while (!finish) {
    interrupted = false;
    char * input_line = readline("> ");

    if (input_line == NULL) {
      finish = true;
      continue;
    }

    std::istringstream iss({std::string(input_line)});
    free(input_line);

    std::string line;
    while (!finish && std::getline(iss, line)) {
      clean_command(line);
      if (line.empty()) {
        continue;
      }
      add_history(line.c_str());
      if (line == "quit") {
        finish = true;
      } else {
        std::ostringstream os;
        process_command(line, os, false);
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
  // Tabs and '\r' (files written on Windows) are spaces
  std::replace_if(
    command.begin(), command.end(), [](char c) {return c == '\t' || c == '\r';}, ' ');

  // remove continuous spaces
  size_t pos = 0;
  while ((pos = command.find("  ")) != command.npos) {
    command.erase(pos, 1);
  }

  // remove from spaces
  while (!command.empty() && *command.begin() == ' ') {
    command.erase(0, 1);
  }

  // remove trailing spaces
  while (!command.empty() && command[command.size() - 1] == ' ') {
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
Terminal::process_get_model_derived_predicate(
  std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    auto deriveds = domain_client_->getDerivedPredicate(command[0]);
    if (deriveds.size() > 0) {
      for (auto derived : deriveds) {
        os << "Parameters: " << derived.predicate.parameters.size() << std::endl;
        for (size_t i = 0; i < derived.predicate.parameters.size(); i++) {
          os << "\t" << derived.predicate.parameters[i].type << " - " <<
            derived.predicate.parameters[i].name << std::endl;
        }
        os << "Preconditions: " << parser::pddl::toString(derived.preconditions) << std::endl;
        os << "\n";
      }
    } else {
      os << "Error when looking for params of " << command[0] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget model derived [predicate_name]" << std::endl;
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
      // An object fluent has objects of a type; the others, numbers
      os << "Returns: " <<
        (functions.value().object.empty() ? "number" : functions.value().object) << std::endl;
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
      os << "Parameters: " << action->parameters.size() << std::endl;
      for (size_t i = 0; i < action->parameters.size(); i++) {
        os << "\t" << action->parameters[i].type << " - " <<
          action->parameters[i].name << std::endl;
      }
      os << "Preconditions: " << parser::pddl::toString(action->preconditions) << std::endl;
      os << "Effects: " << parser::pddl::toString(action->effects) << std::endl;
    } else if (durative_action) {
      os << "Type: durative-action" << std::endl;
      os << "Parameters: " << durative_action->parameters.size() << std::endl;
      for (size_t i = 0; i < durative_action->parameters.size(); i++) {
        os << "\t" << durative_action->parameters[i].name << " - " <<
          durative_action->parameters[i].type << std::endl;
      }
      os << "Duration: " << parser::pddl::toString(durative_action->duration) << std::endl;
      os << "AtStart requirements: " <<
        parser::pddl::toString(durative_action->at_start_requirements) << std::endl;
      os << "OverAll requirements: " <<
        parser::pddl::toString(durative_action->over_all_requirements) << std::endl;
      os << "AtEnd requirements: " <<
        parser::pddl::toString(durative_action->at_end_requirements) << std::endl;
      os << "AtStart effect: " <<
        parser::pddl::toString(durative_action->at_start_effects) << std::endl;
      os << "AtEnd effect: " <<
        parser::pddl::toString(durative_action->at_end_effects) << std::endl;
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
        os << "\t" << predicate.name << std::endl;
      }
    } else if (command[0] == "deriveds") {
      auto predicates = domain_client_->getDerivedPredicates();

      os << "Derived predicates: " << predicates.size() << std::endl;
      for (const auto & predicate : predicates) {
        os << "\t" << predicate.predicate.name << std::endl;
      }
    } else if (command[0] == "functions") {
      auto functions = domain_client_->getFunctions();

      os << "Functions: " << functions.size() << std::endl;
      for (const auto & function : functions) {
        os << "\t" << function.name << std::endl;
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
    } else if (command[0] == "derived") {
      pop_front(command);
      process_get_model_derived_predicate(command, os);
    } else if (command[0] == "function") {
      pop_front(command);
      process_get_model_function(command, os);
    } else if (command[0] == "action") {
      pop_front(command);
      process_get_model_action(command, os);
    } else {
      os <<
        "\tUsage: \n\t\tget model "
        "[types|predicates|functions|actions|predicate|function|action|derived|deriveds]..."
         <<
        std::endl;
    }
  } else {
    os <<
      "\tUsage: \n\t\tget model "
      "[types|predicates|functions|actions|predicate|function|action|derived|deriveds]..."
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
        os << parser::pddl::toString(predicate) << std::endl;
      }
    } else if (command[0] == "functions") {
      auto functions = problem_client_->getFunctions();

      os << "Functions: " << functions.size() << std::endl;
      for (const auto & function : functions) {
        os << parser::pddl::toString(function) << std::endl;
      }
    } else if (command[0] == "goal") {
      auto goal = problem_client_->getGoal();
      os << "Goal: " << parser::pddl::toString(goal) << std::endl;
    } else if (command[0] == "metric") {
      auto metric = parser::pddl::toString(problem_client_->getMetric());
      os << "Metric: " << (metric.empty() ? "none" : metric) << std::endl;
    } else {
      os << "\tUsage: \n\t\tget problem [instances|predicates|functions|goal|metric]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget problem [instances|predicates|functions|goal|metric]..." <<
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
        for (const auto & plan_item : plan.value().items) {
          os << plan_item.time << ":\t" << plan_item.action << "\t[" <<
            plan_item.duration << "]" << std::endl;
        }
      } else {
        os << "Plan not found" << std::endl;
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
    if (!problem_client_->addInstance(plansys2::Instance(command[0], command[1]))) {
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
  auto predicate = parse_predicate(command);
  if (!predicate) {
    os << "\tUsage: \n\t\tset predicate (predicate)" << std::endl;
    return;
  }
  if (!problem_client_->addPredicate(predicate.value())) {
    os << "Could not add the predicate [" << parser::pddl::toString(predicate.value()) << "]" <<
      std::endl;
  }
}

void
Terminal::process_set_function(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    std::string total_expr;
    for (const auto & token : command) {
      total_expr += " " + token;
    }

    plansys2::Function function = parser::pddl::fromStringFunction(total_expr);

    if (!problem_client_->addFunction(function)) {
      os <<
        "Could not add the function [" <<
        parser::pddl::toString(function) << "]" << std::endl;
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

    auto goal = parser::pddl::fromString(total_expr);

    if (!goal.nodes.empty()) {
      if (!problem_client_->setGoal(goal)) {
        os << "Could not set the goal [" << parser::pddl::toString(goal) << "]" << std::endl;
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
Terminal::process_set_metric(std::vector<std::string> & command, std::ostringstream & os)
{
  std::string total_expr;
  for (const auto & token : command) {
    total_expr += " " + token;
  }

  plansys2::Metric metric(total_expr);
  if (metric.tree.nodes.empty()) {
    os << "\tUsage: \n\t\tset metric minimize|maximize <expression>" << std::endl;
  } else if (!problem_client_->setMetric(metric)) {
    os << "Could not set the metric [" << parser::pddl::toString(metric) << "]" << std::endl;
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
    } else if (command[0] == "metric") {
      pop_front(command);
      process_set_metric(command, os);
    } else {
      os << "\tUsage: \n\t\tset [instance|predicate|function|goal|metric]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tset [instance|predicate|function|goal|metric]..." <<
      std::endl;
  }
}

void
Terminal::process_remove_instance(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    if (!problem_client_->removeInstance(plansys2::Instance(command[0]))) {
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
  auto predicate = parse_predicate(command);
  if (!predicate) {
    os << "\tUsage: \n\t\tremove predicate (predicate)" << std::endl;
    return;
  }
  if (!problem_client_->removePredicate(predicate.value())) {
    os << "Could not remove the predicate [" << parser::pddl::toString(predicate.value()) <<
      "]" << std::endl;
  }
}

void
Terminal::process_remove_function(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    plansys2::Function function;
    function.node_type = plansys2_msgs::msg::Node::FUNCTION;

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
      function.parameters.push_back(parser::pddl::fromStringParam(match.str(0)));
      temp = match.suffix().str();
    }

    if (!problem_client_->removeFunction(function)) {
      os << "Could not remove the function [" << parser::pddl::toString(function) << "]" <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove function [name] [instance1] [instance2] ...[instanceN]" <<
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
    } else if (command[0] == "goal") {
      problem_client_->clearGoal();
    } else if (command[0] == "metric") {
      problem_client_->clearMetric();
    } else {
      os << "\tUsage: \n\t\tremove [instance|predicate|function|goal|metric]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove [instance|predicate|function|goal|metric]..." <<
      std::endl;
  }
}

void
Terminal::execute_plan(int items)
{
  auto plan = planner_client_->getPlan(domain_client_->getDomain(), problem_client_->getProblem());

  if (!plan.has_value()) {
    std::cout << "Plan could not be computed " << std::endl;
    return;
  } else if (plan.value().items.size() == 0) {
    std::cout << "Planning succeeded, but there are no actions to execute " << std::endl;
    return;
  }

  if (items > 0 && items <= static_cast<int>(plan.value().items.size())) {
    plan.value().items.erase(plan.value().items.begin() + items, plan.value().items.end());
  } else if (items != -1) {
    std::cout << "Can't execute " << items << " actions" << std::endl;
    return;
  }

  execute_plan(plan.value());
}

void
Terminal::execute_plan(const plansys2_msgs::msg::Plan & plan)
{
  rclcpp::Rate loop_rate(5);


  interrupted = false;
  if (!executor_client_->start_plan_execution(plan)) {
    std::cout << "Execution could not start " << std::endl;
    return;
  }

  while (rclcpp::ok() && executor_client_->execute_and_check_plan()) {
    if (interrupted.exchange(false)) {
      // Ctrl-C: the plan stops, not only the terminal (it kept running in the executor)
      executor_client_->cancel_plan_execution();
      std::cout << std::endl << "Plan cancelled" << std::endl;
      return;
    }

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

    loop_rate.sleep();
  }

  std::cout << std::endl;

  auto plan_result = executor_client_->getResult();
  if (!plan_result.has_value()) {
    std::cout << "Plan execution ended without result." << std::endl;
  } else {
    switch (plan_result.value().result) {
      case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
        std::cout << "Successfully finished" << std::endl;
        break;
      case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
        std::cout << "Preempted " << std::endl;
        break;
      case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
        std::cout << "Finished with error(s)" << std::endl;
        break;
    }
  }
}

void
Terminal::execute_action(std::vector<std::string> & command)
{
  const std::string complete_action = join(command);

  plansys2_msgs::msg::PlanItem action;
  action.time = 0.0;
  action.action = complete_action;
  action.duration = 1.0;

  plansys2_msgs::msg::Plan single_plan;
  single_plan.items.push_back(action);

  execute_plan(single_plan);
}

void
Terminal::process_run(std::vector<std::string> & command, std::ostringstream & os)
{
  const std::string usage =
    "\tUsage: \n\t\trun\n"
    "\tUsage: \n\t\trun num_actions [number of actions to execute from plan]\n"
    "\tUsage: \n\t\trun action [action to execute]\n"
    "\tUsage: \n\t\trun plan-file [planfile]";

  if (command.empty()) {
    execute_plan();
  } else if (command[0] == "action") {
    pop_front(command);
    if (command.empty()) {
      os << "\tUsage: \n\t\trun action [action to execute]" << std::endl;
      return;
    }
    execute_action(command);
  } else if (command[0] == "num_actions") {
    int items = 0;
    const std::string arg = command.size() == 2 ? command[1] : "";
    auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(), items);
    if (arg.empty() || ec != std::errc() || ptr != arg.data() + arg.size() || items <= 0) {
      os << "\tUsage: \n\t\trun num_actions [number of actions to execute from plan]" <<
        std::endl;
      return;
    }
    execute_plan(items);
  } else if (command[0] == "plan-file") {
    if (command.size() != 2) {
      os << "\tUsage: \n\t\trun plan-file [planfile]" << std::endl;
      return;
    }
    std::optional<plansys2_msgs::msg::Plan> plan = parse_plan(command[1]);
    if (!plan.has_value()) {
      os << "Plan could not be loaded " << std::endl;
      return;
    }
    execute_plan(plan.value());
  } else {
    os << usage << std::endl;
  }
}

void
Terminal::process_check_actors(std::vector<std::string> & command, std::ostringstream & os)
{
  (void)command;
  // Filled in the thread that spins the terminal
  auto actors_mutex = std::make_shared<std::mutex>();
  auto heard = std::make_shared<std::map<std::string, plansys2_msgs::msg::ActionPerformerStatus>>();

  auto status_callback =
    [actors_mutex, heard](const plansys2_msgs::msg::ActionPerformerStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(*actors_mutex);
      (*heard)[msg->node_name] = *msg;
    };

  // Relative, as the performers publish it: namespaces and remappings apply
  auto status_sub = create_subscription<plansys2_msgs::msg::ActionPerformerStatus>(
    "performers_status", rclcpp::QoS(100).reliable(), status_callback);

  // Performers publish their status every second: two seconds see them all
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  status_sub.reset();
  std::map<std::string, plansys2_msgs::msg::ActionPerformerStatus> actors;
  {
    std::lock_guard<std::mutex> lock(*actors_mutex);
    actors = *heard;
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
  if (command.size() == 1 && command[0] == "actors") {
    process_check_actors(command, os);
  } else {
    os << "\tUsage: \n\t\tcheck actors" << std::endl;
  }

  // ToDo(fmrico): We should be able to run directly an action, for example:
  // run (move leia entrance dinning)
}

void
Terminal::process_help(std::vector<std::string> & command, std::ostringstream & os)
{
  std::ostringstream cmd_set;
  cmd_set << "\t set instance <object> <type>" << std::endl <<
    "\t set predicate <predicate>" << std::endl <<
    "\t set function (= <function> <value>)" << std::endl <<
    "\t set goal <goal_formula>" << std::endl <<
    "\t set metric minimize|maximize <expression>" << std::endl;
  std::ostringstream cmd_get;
  cmd_get << "\t get model   types | predicates | functions | actions |" << std::endl <<
    "\t             predicate <predicate_name> | function <function_name> |" << std::endl <<
    "\t             action <action_name> | derived <predicate_name> | deriveds" << std::endl <<
    "\t get problem  instances | predicates | functions | goal | metric" << std::endl <<
    "\t get domain" << std::endl <<
    "\t get plan" << std::endl;
  std::ostringstream cmd_rem;
  cmd_rem << "\t remove instance <object>" << std::endl <<
    "\t remove predicate <predicate>" << std::endl <<
    "\t remove function <function>" << std::endl <<
    "\t remove goal" << std::endl <<
    "\t remove metric" << std::endl;

  std::ostringstream cmd_run;
  cmd_run << "\t run action <action>" << std::endl <<
    "\t run num_actions <num>" << std::endl <<
    "\t run plan-file <planfile>" << std::endl;

  std::ostringstream cmd_check;
  cmd_check << "\t check [actors]" << std::endl;

  std::ostringstream cmd_source;
  cmd_source << "\t source <file_name> [0|1]" << std::endl;

  std::ostringstream cmd_help;
  cmd_help << "\t help|? [ set | get | remove | run | check |" <<
    " source | quit | help | ? ]" << std::endl;

  std::map<std::string, std::string> cmds;
  cmds.insert(std::pair<std::string, std::string>("set", cmd_set.str()));
  cmds.insert(std::pair<std::string, std::string>("get", cmd_get.str()));
  cmds.insert(std::pair<std::string, std::string>("remove", cmd_rem.str()));
  cmds.insert(std::pair<std::string, std::string>("run", cmd_run.str()));
  cmds.insert(std::pair<std::string, std::string>("check", cmd_check.str()));
  cmds.insert(std::pair<std::string, std::string>("source", cmd_source.str()));
  cmds.insert(std::pair<std::string, std::string>("quit", std::string("")));
  cmds.insert(std::pair<std::string, std::string>("help", cmd_help.str()));
  cmds.insert(std::pair<std::string, std::string>("?", cmd_help.str()));

  if (command.empty()) {
    os << "The list of available commands is the following:" << std::endl;
    for (auto it1 = cmds.begin(); it1 != cmds.end(); it1++) {
      os << it1->first << std::endl << it1->second << std::endl;
    }
    return;
  }

  const std::string c = command[0];
  auto cmd = cmds.find(c);
  if (cmd != cmds.end()) {
    os << cmd->first << std::endl;
    os << cmd->second << std::endl;
    return;
  }
  os << "Command \"" << command[0] << "\" command not found" << std::endl;
}

void
Terminal::process_source(std::vector<std::string> & command, std::ostringstream & os)
{
  const std::string cmd_help = "\tUsage: \n\t\tsource <filename> [0|1]";

  if (command.empty() || command.size() > 2 || command[0].empty()) {
    os << cmd_help << std::endl;
    return;
  }

  std::string cmdfile = command[0];
  bool do_echo = false;
  if (command.size() == 2) {
    if (command[1] == "0") {
      do_echo = false;
    } else if (command[1] == "1") {
      do_echo = true;
    } else {
      os << cmd_help << std::endl;
      return;
    }
  }

  std::ifstream cmd_ifs(cmdfile, std::ifstream::in);
  if (cmd_ifs.fail()) {
    os << "\tFailing to open file \"" << cmdfile << "\"" << std::endl;
    return;
  }

  bool finish_parsing = false;
  std::string ln;
  while (!finish_parsing && std::getline(cmd_ifs, ln, '\n')) {
    clean_command(ln);
    std::ostringstream os;
    if (do_echo) {os << ln << std::endl;}
    finish_parsing = process_command(ln, os, true);
    std::cout << os.str();
  }

  cmd_ifs.close();
}

bool
Terminal::process_command(std::string & command, std::ostringstream & os, bool inside_source)
{
  bool finish_parsing = false;
  std::vector<std::string> tokens = tokenize(command);

  if (tokens.empty()) {
    return finish_parsing;
  }

  // An error in a command is reported, and the terminal goes on
  try {
    finish_parsing = dispatch_command(tokens, os, inside_source);
  } catch (const std::exception & e) {
    os << "Error: " << e.what() << std::endl;
  }
  return finish_parsing;
}

bool
Terminal::dispatch_command(
  std::vector<std::string> & tokens, std::ostringstream & os, bool inside_source)
{
  bool finish_parsing = false;
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
  } else if ((tokens[0] == "help") || (tokens[0] == "?")) {
    pop_front(tokens);
    process_help(tokens, os);
  } else if (tokens[0] == "source") {
    if (!inside_source) {
      pop_front(tokens);
      process_source(tokens, os);
    } else {
      os << "Nested \"source\" commands not allowed" << std::endl;
    }
    finish_parsing = true;
  } else if (tokens[0] == "quit") {
    finish_parsing = true;
  } else {
    os << "Command not found" << std::endl;
  }
  return finish_parsing;
}

}  // namespace plansys2_terminal
