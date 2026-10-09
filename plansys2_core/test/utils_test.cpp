// Copyright 2029 Intelligent Robotics Lab
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

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_path.hpp"
#include "plansys2_core/PlanSolverBase.hpp"
#include "plansys2_core/Utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

TEST(utils_test, functions)
{
  std::string st1("this is a message");
  std::vector<std::string> est1 = {"this", "is", "a", "message"};
  std::vector<std::string> est1b = {"this", "a", "message"};
  auto res1 = plansys2::tokenize(st1, " ");

  ASSERT_EQ(res1, est1);
  ASSERT_NE(res1, est1b);

  std::string st2("this:is:a:message");
  std::vector<std::string> est2 = {"this", "is", "a", "message"};
  auto res2 = plansys2::tokenize(st2, ":");

  ASSERT_EQ(res2, est2);

  std::string st3("this       is a     message");
  std::vector<std::string> est3 = {"this", "is", "a", "message"};
  auto res3 = plansys2::tokenize(st3, " ");

  ASSERT_EQ(res3, est3);
}

using namespace std::chrono_literals;  // NOLINT

class PlannerTest : public plansys2::PlanSolverBase
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, const std::string & plugin_name)
  {
    lc_node_ = lc_node;
  }

  std::vector<std::string> tokenize_test(const std::string & command)
  {
    char ** argv = tokenize(command);
    std::vector<std::string> result;

    for (int i = 0; argv[i] != nullptr; ++i) {
      result.emplace_back(argv[i]);
    }

    return result;
  }

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "", const rclcpp::Duration solver_timeoutt = 15s)
  {
    return {};
  }

  bool isDomainValid(const std::string & domain, const std::string & node_namespace)
  {
    return true;
  }
};

TEST(utils_test, tokenizer_tests)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  PlannerTest planner;
  planner.configure(node, "test_node");

  auto ret_1 = planner.tokenize_test("myplanner");
  ASSERT_EQ(ret_1.size(), 1u);
  ASSERT_EQ(ret_1, std::vector<std::string>({"myplanner"}));

  auto ret_2 = planner.tokenize_test("./myplanner");
  ASSERT_EQ(ret_2.size(), 1u);
  ASSERT_EQ(ret_2, std::vector<std::string>({"./myplanner"}));

  auto ret_3 = planner.tokenize_test("myplanner subcmd1");
  ASSERT_EQ(ret_3.size(), 2u);
  ASSERT_EQ(ret_3, std::vector<std::string>({"myplanner", "subcmd1"}));

  auto ret_4 = planner.tokenize_test("myplanner   subcmd1  ");
  ASSERT_EQ(ret_4.size(), 2u);
  ASSERT_EQ(ret_4, std::vector<std::string>({"myplanner", "subcmd1"}));

  auto ret_5 = planner.tokenize_test("myplanner subcmd1 subcmd2");
  ASSERT_EQ(ret_5.size(), 3u);
  ASSERT_EQ(ret_5, std::vector<std::string>({"myplanner", "subcmd1", "subcmd2"}));

  auto ret_6 = planner.tokenize_test("myplanner subcmd1 subcmd2 subcmd3");
  ASSERT_EQ(ret_6.size(), 4u);
  ASSERT_EQ(ret_6, std::vector<std::string>({"myplanner", "subcmd1", "subcmd2", "subcmd3"}));

  auto ret_7 = planner.tokenize_test("myplanner subcmd1 subcmd2 subcmd3 subcmd4");
  ASSERT_EQ(ret_7.size(), 5u);
  ASSERT_EQ(
    ret_7, std::vector<std::string>({"myplanner", "subcmd1", "subcmd2", "subcmd3", "subcmd4"}));
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
