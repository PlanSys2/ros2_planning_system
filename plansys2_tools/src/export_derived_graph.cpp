// Copyright 2022 Intelligent Robotics Lab
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

// #include "rclcpp/rclcpp.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);

  if (argc != 3) {
    std::cerr << "Usage: export_graph <domain_file.pddl> <graph_file.dot>\n";
    return 1;
  }

  std::string domain_file = argv[1];
  std::string graph_file = argv[2];

  std::ifstream domain_ifs(domain_file);
  if (!domain_ifs.is_open()) {
    std::cerr << "Failed to open domain file: " << domain_file << "\n";
    return 1;
  }

  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  auto graph = domain_expert.getDerivedResolutionGraph();
  graph.exportToDOT(graph_file);

  // rclcpp::shutdown();
  return 0;
}