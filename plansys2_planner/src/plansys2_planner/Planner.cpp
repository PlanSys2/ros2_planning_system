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

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_planner/Planner.hpp"

#define SIZE (10 * 1024)

namespace plansys2
{

Planner::Planner()
{
}

std::optional<Plan>
Planner::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    //  This doesn't work as cxx flags must apper at end of link options, and I didn't
    //  find a way
    // std::experimental::filesystem::create_directories("/tmp/" + node_namespace);
    mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
  }

  Plan ret;
  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();

  system(("ros2 run popf popf /tmp/" + node_namespace + "/domain.pddl /tmp/" +
    node_namespace + "/problem.pddl > /tmp/" + node_namespace + "/plan").c_str());

  std::string line;
  std::ifstream plan_file("/tmp/" + node_namespace + "/plan");
  bool solution = false;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      } else if (line.front() != ';') {
        PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 1);
        duration.pop_back();

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);

        ret.push_back(item);
      }
    }
    plan_file.close();
  }

  if (ret.empty()) {
    return {};
  } else {
    return ret;
  }
}

}  // namespace plansys2
