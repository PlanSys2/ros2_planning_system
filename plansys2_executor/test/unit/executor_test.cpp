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

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "plansys2_executor/ActionExecutor.hpp"

#include "gtest/gtest.h"

class ActionExecutorTest : public plansys2::ActionExecutor
{
public:
  std::string test_get_name(const std::string & action_expr)
  {
    return get_name(action_expr);
  }

  std::vector<std::string> test_get_params(const std::string & action_expr)
  {
    return get_params(action_expr);
  }
};

TEST(problem_expert, util_test)
{
  ActionExecutorTest action_executor;
  std::string action_1("(move r2d2 bedroom kitchen)");

  ASSERT_EQ(action_executor.test_get_name(action_1), "move");
  auto params_1 = action_executor.test_get_params(action_1);
  ASSERT_EQ(params_1.size(), 3);
  ASSERT_EQ(params_1[0], "r2d2");
  ASSERT_EQ(params_1[1], "bedroom");
  ASSERT_EQ(params_1[2], "kitchen");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
