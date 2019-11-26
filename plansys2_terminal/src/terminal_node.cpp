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


class Terminal : public rclcpp::Node
{
public:
  Terminal()
  : rclcpp::Node("terminal")
  {
  }

  void run_console()
  {
    std::shared_ptr<rclcpp::Node> domain_expert_client = shared_from_this();
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>(domain_expert_client);

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

  void process_predicate(const std::vector<std::string> & command)
  {
    if (command.size() == 3) {
      auto predicates = domain_client_->getPredicate(command[2]);
      if (predicates.has_value()) {
        std::cout << "Parameters: " << predicates.value().parameters.size() << std::endl;
        for (size_t i = 0; i < predicates.value().parameters.size(); i++) {
          std::cout << "\t" << predicates.value().parameters[i].type << " - " <<
            predicates.value().parameters[i].name << std::endl;
        }
      } else {
        std::cout << "Error when looking for params of " << command[2] << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget predicate [predicate_name]" << std::endl;
    }
  }

  void process_action(const std::vector<std::string> & command)
  {
    if (command.size() == 3) {
      auto action = domain_client_->getAction(command[2]);
      auto durative_action = domain_client_->getDurativeAction(command[2]);
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
        std::cout << "Error when looking for params of " << command[2] << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget action [action_name]" << std::endl;
    }
  }

  void process_get(const std::vector<std::string> & command)
  {
    if (command.size() > 1) {
      if (command[1] == "types") {
        auto types = domain_client_->getTypes();

        std::cout << "Types: " << types.size() << std::endl;
        for (const auto & type : types) {
          std::cout << "\t" << type << std::endl;
        }
      } else if (command[1] == "predicates") {
        auto predicates = domain_client_->getPredicates();

        std::cout << "Predicates: " << predicates.size() << std::endl;
        for (const auto & predicate : predicates) {
          std::cout << "\t" << predicate << std::endl;
        }
      } else if (command[1] == "actions") {
        auto actions = domain_client_->getActions();
        auto durative_actions = domain_client_->getDurativeActions();

        std::cout << "Actions: " << actions.size() << std::endl;
        for (const auto & action : actions) {
          std::cout << "\t" << action << " (action)" << std::endl;
        }
        for (const auto & durative_action : durative_actions) {
          std::cout << "\t" << durative_action << " (durative action)" << std::endl;
        }
      } else if (command[1] == "predicate") {
        process_predicate(command);
      } else if (command[1] == "action") {
        process_action(command);
      } else {
        std::cout << "\tUsage: \n\t\tget [types|predicates|actions|predicate|action]..." <<
          std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tget [types|predicates|actions|predicate|action]..." <<
        std::endl;
    }
  }

  void process_command(const std::string & command)
  {
    std::vector<std::string> tokens = tokenize(command);

    if (tokens.empty()) {
      return;
    }

    if (tokens[0] == "get") {
      process_get(tokens);
    } else {
      std::cout << "Command not found" << std::endl;
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto terminal_node = std::make_shared<Terminal>();

  terminal_node->run_console();

  rclcpp::shutdown();
  return 0;
}
