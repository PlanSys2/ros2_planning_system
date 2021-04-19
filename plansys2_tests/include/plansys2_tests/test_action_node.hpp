// Copyright 2021 Intelligent Robotics Lab
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

#ifndef PLANSYS2_TESTS__TEST_ACTION_NODE_HPP_
#define PLANSYS2_TESTS__TEST_ACTION_NODE_HPP_

#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_tests
{

using namespace std::chrono_literals;  // NOLINT (build/namespaces)

class TestAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<TestAction>;
  static Ptr make_shared(
    const std::string & action,
    const std::chrono::seconds & rate = 1s, float increment = 0.4)
  {
    auto ret = std::make_shared<TestAction>(action, rate, increment);
    ret->set_parameter(rclcpp::Parameter("action_name", action));
    ret->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    return ret;
  }

  TestAction(
    const std::string & action_name,
    const std::chrono::seconds & rate = 1s,
    float increment = 0.4);

private:
  void do_work();

  static int node_name_counter_;
  float progress_;
  float increment_;
};

}  // namespace plansys2_tests

#endif  // PLANSYS2_TESTS__TEST_ACTION_NODE_HPP_
