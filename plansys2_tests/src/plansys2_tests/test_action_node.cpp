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

#include "plansys2_tests/test_action_node.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"


namespace plansys2_tests
{

int TestAction::node_name_counter_ = 0;

TestAction::TestAction(
  const std::string & action_name, const std::chrono::seconds & rate, float increment)
: plansys2::ActionExecutorClient(action_name + std::to_string(node_name_counter_++),
    rate), increment_(increment)
{
  progress_ = 0.0;
}

void
TestAction::do_work()
{
  if (progress_ < 1.0) {
    progress_ += increment_;
    send_feedback(progress_, "Action running");
  } else {
    finish(true, 1.0, "Action completed");
    progress_ = 0.0;
  }
}

}  // namespace plansys2_tests
