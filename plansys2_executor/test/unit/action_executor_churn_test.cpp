// Copyright 2026 Intelligent Robotics Lab
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

// ActionExecutors are created and destroyed in one thread while their node spins in another,
// as ExecutorNode does with each plan: nothing may crash or hang

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using Status = plansys2::ActionExecutor::Status;

namespace
{

class QuickMove : public plansys2::ActionExecutorClient
{
public:
  explicit QuickMove(const std::string & name)
  : ActionExecutorClient(name)
  {
    set_parameter({"action_name", "move"});
    set_parameter({"rate", 50.0});
  }

private:
  void do_work() override
  {
    if (++cycles_ >= 2) {
      cycles_ = 0;
      finish(true, 1.0, "done");
    } else {
      send_feedback(0.5, "moving");
    }
  }

  int cycles_ {0};
};

}  // namespace

TEST(ActionExecutorChurn, create_and_destroy_while_spinning)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("executor");
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  // Kept, as ExecutorNode keeps it: the executors come and go, not the hub (with Zenoh, a
  // subscription destroyed while the process publishes may deadlock in rmw_zenoh)
  auto hub = plansys2::ActionHub::get(node);
  auto performer = std::make_shared<QuickMove>("move_performer");

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(performer->get_node_base_interface());
  plansys2::testing::ExecutorThread spinner(exe);
  performer->trigger_transition(Transition::TRANSITION_CONFIGURE);
  // The performer listens and answers on the actions hub
  ASSERT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        return node->count_subscribers("/actions_hub") >= 1u &&
               node->count_publishers("/actions_hub") >= 1u;
      }));

  const char * env = std::getenv("PLANSYS2_TEST_CHURN_ROUNDS");
  const int rounds = env ? std::atoi(env) : 200;
  std::mt19937 random(42);
  int ran = 0;
  for (int i = 0; i < rounds; i++) {
    auto executor = plansys2::ActionExecutor::make_shared("(move r2d2 a b)", node);
    executor->tick(node->now());
    // Sometimes it gets its performer, or even ends, before it is dropped
    auto wait = std::chrono::milliseconds(random() % 60);
    auto deadline = std::chrono::steady_clock::now() + wait;
    while (std::chrono::steady_clock::now() < deadline) {
      executor->tick(node->now());
      std::this_thread::sleep_for(2ms);
    }
    ran += executor->get_internal_status() != Status::DEALING;
    if (random() % 2) {
      executor->cancel();
    }
    executor->clean_up();
    executor.reset();
  }

  // The performer was reached in part of the rounds, not only asked
  EXPECT_GT(ran, 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
