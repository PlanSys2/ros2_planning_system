// Copyright 2022 Marco Roveri - University of Trento
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

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "plansys2_pddl_parser/Instance.h"
#include "gtest/gtest.h"

using namespace std::chrono_literals;

class PDDLParserTestCase : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST(PDDLParserTestCase, pddl_parser)
{

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_pddl_parser");
  std::string domain_file = pkgpath + "/pddl/dom1.pddl";
  std::string instance_file = pkgpath + "/pddl/prob1.pddl";

  std::ifstream domain_ifs(domain_file);
  ASSERT_TRUE(domain_ifs.good());
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
                         std::istreambuf_iterator<char>());
  ASSERT_NE(domain_str, "");
  std::ifstream instance_ifs(instance_file);
  ASSERT_TRUE(instance_ifs.good());
  std::string instance_str((
      std::istreambuf_iterator<char>(instance_ifs)),
                           std::istreambuf_iterator<char>());

  ASSERT_NE(instance_str, "");
  // Read domain and instance
  bool okparse = false;
  bool okprint = false;
  try {
    parser::pddl::Domain domain( domain_str );
    parser::pddl::Instance instance( domain, instance_str );
    okparse = true;
    try {
      std::cout << domain << std::endl;
      std::cout << instance << std::endl;
      okprint = true;
    } catch (std::runtime_error e) {
    }
  } catch (std::runtime_error e) {
  }
  ASSERT_TRUE(okparse);
  ASSERT_TRUE(okprint);
}
