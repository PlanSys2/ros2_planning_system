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

#include <vector>
#include <list>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"

std::vector<std::string> tokenize(const std::string & text)
{
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
  tokens.erase(tokens.begin(), tokens.begin() + 1);
}

class Terminal : public rclcpp::Node
{
public:
  Terminal()
  : rclcpp::Node("terminal")
  {
  }

  void run_console()
  {
    std::shared_ptr<rclcpp::Node> terminal_node = shared_from_this();
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>(terminal_node);
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(terminal_node);
    planner_client_ = std::make_shared<plansys2::PlannerClient>(terminal_node);
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(terminal_node);

    std::string line;
    bool success = true;

    std::cout << "ROS2 Planning System console. Type \"quit\" to finish" << std::endl;
    std::cout << "> ";
    while (std::getline(std::cin, line)) {
      if (line == "quit") {
        break;
      }

      process_command(line);
      std::cout << "> ";
    }

    std::cout << "Finishing..." << std::endl;
  }

  void process_get_mode_predicate(std::vector<std::string> & command)
  {
    if (command.size() == 1) {
      auto predicates = domain_client_->getPredicate(command[0]);
      if (predicates.has_value()) {
        std::cout << "Parameters: " << predicates.value().parameters.size() << std::endl;
        for (size_t i = 0; i < predicates.value().parameters.size(); i++) {
          std::cout << "\t" << predicates.value().parameters[i].type << " - " <<
            predicates.value().parameters[i].name << std::endl;
        }
      } else {
        std::cout << "Error when looking for params of " << command[0] << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget mode predicate [predicate_name]" << std::endl;
    }
  }

  void process_get_mode_action(std::vector<std::string> & command)
  {
    if (command.size() == 1) {
      auto action = domain_client_->getAction(command[0]);
      auto durative_action = domain_client_->getDurativeAction(command[0]);
      if (action.has_value()) {
        std::cout << "Type: action" << std::endl;
        std::cout << "Parameters: " << action.value().parameters.size() << std::endl;
        for (size_t i = 0; i < action.value().parameters.size(); i++) {
          std::cout << "\t" << action.value().parameters[i].type << " - " <<
            action.value().parameters[i].name << std::endl;
        }
        std::cout << "Preconditions: " << action.value().preconditions.toString() << std::endl;
        std::cout << "Effects: " << action.value().effects.toString() << std::endl;
      } else if (durative_action.has_value()) {
        std::cout << "Type: durative-action" << std::endl;
        std::cout << "Parameters: " << durative_action.value().parameters.size() << std::endl;
        for (size_t i = 0; i < durative_action.value().parameters.size(); i++) {
          std::cout << "\t" << durative_action.value().parameters[i].name << " - " <<
            durative_action.value().parameters[i].type << std::endl;
        }
        std::cout << "AtStart requirements: " <<
          durative_action.value().at_start_requirements.toString() << std::endl;
        std::cout << "OverAll requirements: " <<
          durative_action.value().over_all_requirements.toString() << std::endl;
        std::cout << "AtEnd requirements: " <<
          durative_action.value().at_end_requirements.toString() << std::endl;
        std::cout << "AtStart effect: " <<
          durative_action.value().at_start_effects.toString() << std::endl;
        std::cout << "AtEnd effect: " <<
          durative_action.value().at_end_effects.toString() << std::endl;
      } else {
        std::cout << "Error when looking for params of " << command[0] << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget mode action [action_name]" << std::endl;
    }
  }

  void process_get_mode(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      if (command[0] == "types") {
        auto types = domain_client_->getTypes();

        std::cout << "Types: " << types.size() << std::endl;
        for (const auto & type : types) {
          std::cout << "\t" << type << std::endl;
        }
      } else if (command[0] == "predicates") {
        auto predicates = domain_client_->getPredicates();

        std::cout << "Predicates: " << predicates.size() << std::endl;
        for (const auto & predicate : predicates) {
          std::cout << "\t" << predicate << std::endl;
        }
      } else if (command[0] == "actions") {
        auto actions = domain_client_->getActions();
        auto durative_actions = domain_client_->getDurativeActions();

        std::cout << "Actions: " << actions.size() << std::endl;
        for (const auto & action : actions) {
          std::cout << "\t" << action << " (action)" << std::endl;
        }
        for (const auto & durative_action : durative_actions) {
          std::cout << "\t" << durative_action << " (durative action)" << std::endl;
        }
      } else if (command[0] == "predicate") {
        pop_front(command);
        process_get_mode_predicate(command);
      } else if (command[0] == "action") {
        pop_front(command);
        process_get_mode_action(command);
      } else {
        std::cerr << " mode ---> " << command[0] << std::endl;
        std::cout <<
          "\tUsage: \n\t\tget mode [types|predicates|actions|predicate|action]..." <<
          std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget mode [types|predicates|actions|predicate|action]..." <<
        std::endl;
    }
  }

  void process_get_problem(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      if (command[0] == "instances") {
        auto instances = problem_client_->getInstances();

        std::cout << "Instances: " << instances.size() << std::endl;
        for (const auto & instance : instances) {
          std::cout << "\t" << instance.name << "\t" << instance.type << std::endl;
        }
      } else if (command[0] == "predicates") {
        auto predicates = problem_client_->getPredicates();

        std::cout << "Predicates: " << predicates.size() << std::endl;
        for (const auto & predicate : predicates) {
          std::cout << predicate.toString() << std::endl;
        }
      } else if (command[0] == "goal") {
        auto goal = problem_client_->getGoal();
        std::cout << "Goal: " << goal.toString() << std::endl;
      } else if (command[0] == "problem") {
        auto problem = problem_client_->getProblem();
        std::cout << "Problem: " << problem << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget problem [instances|predicates]..." <<
        std::endl;
    }
  }

  void process_get(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      if (command[0] == "model") {
        pop_front(command);
        process_get_mode(command);
      } else if (command[0] == "problem") {
        pop_front(command);
        process_get_problem(command);
      } else if (command[0] == "domain") {
        std::cout << "domain: \n" << domain_client_->getDomain() << std::endl;
      } else if (command[0] == "plan") {
        auto plan = planner_client_->getPlan(domain_client_->getDomain(),
            problem_client_->getProblem());

        if (plan.has_value()) {
          std::cout << "plan: " << std::endl;
          for (const auto & action : plan.value()) {
            std::cout << action.time << "\t" << action.action << "\t" <<
              action.duration << std::endl;
          }
        } else {
          std::cout << "No se ha encontrado plan" << std::endl;
        }
      } else {
        std::cerr << " get ---> " << command[0] << std::endl;
        std::cout << "\tUsage: \n\t\tget [model|problem|domain]..." <<
          std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget [model|problem|domain|plan]..." <<
        std::endl;
    }
  }

  void process_set_instance(std::vector<std::string> & command)
  {
    if (command.size() == 2) {
      if (!problem_client_->addInstance(plansys2::Instance{command[0], command[1]})) {
        std::cerr << "Could not add the instance [" << command[0] << "]" << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tset instance [name] [type]" <<
        std::endl;
    }
  }

  void process_set_predicate(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      plansys2::Predicate predicate;
      predicate.name = command[0];

      if (predicate.name.front() != '(') {
        std::cout << "\tUsage: \n\t\tset predicate (predicate)" <<
          std::endl;
        return;
      }

      predicate.name.erase(0, 1);  // Remove first )

      pop_front(command);
      while (!command.empty()) {
        plansys2::Param param {command[0], ""};
        predicate.parameters.push_back(param);
        pop_front(command);
      }

      if (predicate.parameters.back().name.back() != ')') {
        std::cout << "\tUsage: \n\t\tset predicate (predicate)" <<
          std::endl;
        return;
      }

      predicate.parameters.back().name.pop_back();  // Remove last (


      if (!problem_client_->addPredicate(predicate)) {
        std::cerr << "Could not add the predicate [" << predicate.toString() << "]" << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tset predicate [predicate]" <<
        std::endl;
    }
  }

  void process_set_goal(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      plansys2::Goal goal;

      std::string total_expr;
      for (const auto & token : command) {
        total_expr += " " + token;
      }

      goal.fromString(total_expr);

      if (goal.root_ != nullptr) {
        if (!problem_client_->setGoal(goal)) {
          std::cerr << "Could not set the goal [" << goal.toString() << "]" << std::endl;
        }
      } else {
        std::cout << "\tUsage: \n\t\tset goal [goal]" <<
          std::endl;
      }
    } else {
      std::cerr << "Not valid goal" << std::endl;
    }
  }
  void process_set(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      if (command[0] == "instance") {
        pop_front(command);
        process_set_instance(command);
      } else if (command[0] == "predicate") {
        pop_front(command);
        process_set_predicate(command);
      } else if (command[0] == "goal") {
        pop_front(command);
        process_set_goal(command);
      } else {
        std::cout << "\tUsage: \n\t\tset [instance|predicate|goal]..." <<
          std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tset [instance|predicate|goal]..." <<
        std::endl;
    }
  }

  void process_remove_instance(std::vector<std::string> & command)
  {
    if (command.size() == 1) {
      if (!problem_client_->removeInstance(command[0])) {
        std::cerr << "Could not remove the instance [" << command[0] << "]" << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tremove instance [name]" <<
        std::endl;
    }
  }

  void process_remove_predicate(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      plansys2::Predicate predicate;
      predicate.name = command[0];

      if (predicate.name.front() != '(') {
        std::cout << "\tUsage: \n\t\tremove predicate (predicate)" <<
          std::endl;
      }

      predicate.name.erase(0, 1);  // Remove first )

      pop_front(command);
      while (!command.empty()) {
        plansys2::Param param {command[0], ""};
        predicate.parameters.push_back(param);
        pop_front(command);
      }


      if (predicate.parameters.back().name.back() != ')') {
        std::cout << "\tUsage: \n\t\tremove predicate (predicate)" <<
          std::endl;
        return;
      }

      predicate.parameters.back().name.pop_back();  // Remove last (

      if (!problem_client_->removePredicate(predicate)) {
        std::cerr << "Could not remove the predicate [" << predicate.toString() << "]" << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\remove predicate [name] [instance1] [instance2] ...[instaceN]" <<
        std::endl;
    }
  }

  void process_remove(std::vector<std::string> & command)
  {
    if (command.size() > 0) {
      if (command[0] == "instance") {
        pop_front(command);
        process_remove_instance(command);
      } else if (command[0] == "predicate") {
        pop_front(command);
        process_remove_predicate(command);
      } else if (command[0] == "goal") {
        problem_client_->clearGoal();
      } else {
        std::cout << "\tUsage: \n\t\tremove [instance|predicate|goal]..." <<
          std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tremove [instance|predicate|goal]..." <<
        std::endl;
    }
  }

  void execute_plan()
  {
    if (executor_client_->executePlan()) {
      rclcpp::Rate loop_rate(5);
      while (rclcpp::ok() && !executor_client_->getResult().has_value()) {
        auto feedback = executor_client_->getFeedBack();

        std::cout << "\r\e[K" << std::flush;
        std::cout << "[" << feedback.seq_action << "/" << feedback.total_actions << "]" <<
          "{" << feedback.current_action << "} [" << feedback.progress_current_action << "%]" <<
          std::flush;

        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
      }

      std::cout << std::endl;

      if (executor_client_->getResult().value().success) {
        std::cout << "Successful finished " << std::endl;
      } else {
        std::cout << "Finished with error: " << executor_client_->getResult().value().error_info <<
          std::endl;
      }
    } else {
      std::cout << "Rejected execution. is there any plan?" << std::endl;
    }
  }


  void process_run(std::vector<std::string> & command)
  {
    if (command.size() == 0) {
      execute_plan();
    } else {
      std::string action(command[0]);
      for (int i = 1; i < command.size(); i++) {
        action += " " + command[i];
      }

      auto action_executor = std::make_shared<plansys2::ActionExecutor>(action);

      rclcpp::Rate loop_rate(5);
      while (rclcpp::ok() && !action_executor->finished()) {
        action_executor->update();
        std::cout << "\r\e[K" << std::flush;
        std::cout << action << "... [" << action_executor->getProgress() << "%]  " << std::flush;
        loop_rate.sleep();
      }
      std::cout << std::endl;

      if (action_executor->getStatus() != plansys2::ActionExecutor::SUCCEDED) {
        std::cout << "Error while executing action" << std::endl;
      } else {
        std::cout << "Action succeded" << std::endl;
      }
    }
  }


  void process_command(std::string & command)
  {
    std::vector<std::string> tokens = tokenize(command);

    if (tokens.empty()) {
      return;
    }

    if (tokens[0] == "get") {
      pop_front(tokens);
      process_get(tokens);
    } else if (tokens[0] == "set") {
      pop_front(tokens);
      process_set(tokens);
    } else if (tokens[0] == "remove") {
      pop_front(tokens);
      process_remove(tokens);
    } else if (tokens[0] == "run") {
      pop_front(tokens);
      process_run(tokens);
    } else {
      std::cout << "Command not found" << std::endl;
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto terminal_node = std::make_shared<Terminal>();

  terminal_node->run_console();

  rclcpp::shutdown();
  return 0;
}
