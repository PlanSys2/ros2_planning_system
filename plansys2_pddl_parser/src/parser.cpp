// Copyright 2024 Intelligent Robotics Lab
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
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "plansys2_pddl_parser/Instance.hpp"

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::cout << "Usage: parser <domain.pddl> <task.pddl>\n";
    exit(1);
  }
  std::ifstream domain_ifs(argv[1]);
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  std::ifstream instance_ifs(argv[2]);
  std::string instance_str(
    (std::istreambuf_iterator<char>(instance_ifs)), std::istreambuf_iterator<char>());

  // Read multiagent domain and instance
  parser::pddl::Domain domain(domain_str);

  std::cout << domain << std::endl;

  parser::pddl::Instance instance(domain, instance_str);

  std::cout << instance << std::endl;
}
