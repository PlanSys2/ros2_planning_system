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

// The lifecycle manager, once PlanSys2 is up: components that start again are brought up, one
// gone for longer than the grace deactivates the executor until it is back

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT
using State = lifecycle_msgs::msg::State;
using Transition = lifecycle_msgs::msg::Transition;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{

const std::vector<std::string> kNames = {"domain_expert", "problem_expert", "planner",
  "executor"};

// A component: a lifecycle node that may refuse to be configured
class Component : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Component(const std::string & name, bool fail_configure = false)
  : LifecycleNode(name), fail_configure_(fail_configure) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    configures++;
    return fail_configure_ ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  std::atomic<int> configures {0};

private:
  bool fail_configure_;
};

}  // namespace

class SupervisorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    exe_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    for (const auto & name : kNames) {
      add(name);
    }
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(*exe_);

    manager_ = rclcpp::Node::make_shared("supervisor_test_manager");
    auto client_node = std::make_shared<plansys2::ClientNode>(*manager_);
    for (const auto & name : kNames) {
      clients_[name] = std::make_shared<plansys2::LifecycleServiceClient>(client_node, name);
    }
    ASSERT_TRUE(plansys2::startup_function(clients_, 3s));

    plansys2::Supervisor::Options options;
    options.lost_after = 500ms;
    options.grace = 1500ms;
    options.transition_timeout = 2s;
    supervisor_ = std::make_unique<plansys2::Supervisor>(clients_, options);
    watching_ = std::thread([this]() {
          while (!stop_) {
            supervisor_->step();
            std::this_thread::sleep_for(100ms);
          }
        });
  }

  void TearDown() override
  {
    stop_ = true;
    if (watching_.joinable()) {
      watching_.join();
    }
    supervisor_.reset();
    clients_.clear();
    spinner_.reset();
    exe_.reset();
    nodes_.clear();
  }

  void add(const std::string & name, bool fail_configure = false)
  {
    nodes_[name] = std::make_shared<Component>(name, fail_configure);
    exe_->add_node(nodes_[name]->get_node_base_interface());
  }

  // Its process dies: nobody answers for it. With the executor stopped, as it may be serving
  // the node right now
  void kill(const std::string & name)
  {
    const bool spinning = spinner_ != nullptr;
    spinner_.reset();
    exe_->remove_node(nodes_[name]->get_node_base_interface());
    nodes_.erase(name);
    if (spinning) {
      spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(*exe_);
    }
  }

  uint8_t state(const std::string & name)
  {
    return nodes_.count(name) ? nodes_[name]->get_current_state().id() :
           State::PRIMARY_STATE_UNKNOWN;
  }

  bool becomes(const std::string & name, uint8_t expected, std::chrono::seconds timeout = 5s)
  {
    return plansys2::testing::wait_for([&]() {return state(name) == expected;}, timeout);
  }

  // Whether the executor stays active the whole time
  bool executor_stays_active(std::chrono::milliseconds time)
  {
    const auto end = std::chrono::steady_clock::now() + time;
    while (std::chrono::steady_clock::now() < end) {
      if (state("executor") != State::PRIMARY_STATE_ACTIVE) {
        return false;
      }
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  std::map<std::string, std::shared_ptr<Component>> nodes_;
  rclcpp::Node::SharedPtr manager_;
  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> clients_;
  std::unique_ptr<plansys2::Supervisor> supervisor_;
  std::thread watching_;
  std::atomic<bool> stop_ {false};
};

TEST_F(SupervisorTest, nothing_to_do_when_all_is_up)
{
  EXPECT_TRUE(executor_stays_active(2s));
  for (const auto & name : kNames) {
    EXPECT_EQ(state(name), State::PRIMARY_STATE_ACTIVE) << name;
    EXPECT_EQ(nodes_[name]->configures.load(), 1) << name;
  }
  EXPECT_FALSE(supervisor_->degraded());
}

TEST_F(SupervisorTest, a_component_that_starts_again_is_brought_up)
{
  for (const std::string name : {"problem_expert", "domain_expert", "planner"}) {
    SCOPED_TRACE(name);
    kill(name);
    add(name);  // At once: as respawn, faster than lost_after
    EXPECT_TRUE(becomes(name, State::PRIMARY_STATE_ACTIVE));
    EXPECT_FALSE(supervisor_->degraded());
    EXPECT_EQ(state("executor"), State::PRIMARY_STATE_ACTIVE);
  }
}

TEST_F(SupervisorTest, a_short_outage_does_not_touch_the_executor)
{
  kill("problem_expert");
  std::this_thread::sleep_for(800ms);  // Lost, but not for the grace
  add("problem_expert");
  EXPECT_TRUE(becomes("problem_expert", State::PRIMARY_STATE_ACTIVE));
  EXPECT_TRUE(executor_stays_active(2s));
  EXPECT_FALSE(supervisor_->degraded());
}

TEST_F(SupervisorTest, gone_for_longer_than_the_grace_deactivates_the_executor)
{
  kill("problem_expert");
  // Lost after 0.5 s, the grace 1.5 s more: the executor is deactivated, and stays
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_INACTIVE, 5s));
  EXPECT_TRUE(supervisor_->degraded());
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(state("executor"), State::PRIMARY_STATE_INACTIVE);

  // Back: brought up, and the executor with it
  add("problem_expert");
  EXPECT_TRUE(becomes("problem_expert", State::PRIMARY_STATE_ACTIVE));
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_ACTIVE));
  EXPECT_FALSE(supervisor_->degraded());
}

TEST_F(SupervisorTest, two_components_gone_the_executor_waits_for_both)
{
  kill("problem_expert");
  kill("domain_expert");
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_INACTIVE, 5s));
  add("problem_expert");
  EXPECT_TRUE(becomes("problem_expert", State::PRIMARY_STATE_ACTIVE));
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(state("executor"), State::PRIMARY_STATE_INACTIVE);  // The domain expert is not
  add("domain_expert");
  EXPECT_TRUE(becomes("domain_expert", State::PRIMARY_STATE_ACTIVE));
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_ACTIVE));
}

TEST_F(SupervisorTest, an_executor_that_starts_again_is_brought_up)
{
  kill("executor");
  add("executor");
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_ACTIVE));
  EXPECT_FALSE(supervisor_->degraded());
}

TEST_F(SupervisorTest, an_executor_restarted_while_degraded_waits)
{
  kill("problem_expert");
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_INACTIVE, 5s));
  kill("executor");
  add("executor");
  // Configured, but not active while the problem expert is gone
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_INACTIVE));
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(state("executor"), State::PRIMARY_STATE_INACTIVE);
  add("problem_expert");
  EXPECT_TRUE(becomes("executor", State::PRIMARY_STATE_ACTIVE));
}

TEST_F(SupervisorTest, what_someone_deactivates_is_left_so)
{
  nodes_["planner"]->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  EXPECT_TRUE(executor_stays_active(2s));
  EXPECT_EQ(state("planner"), State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(supervisor_->degraded());
}

TEST_F(SupervisorTest, a_component_that_cannot_be_configured)
{
  kill("planner");
  add("planner", true);
  // Tried, again and again, without stopping the watch of the others
  EXPECT_TRUE(
    plansys2::testing::wait_for([&]() {return nodes_["planner"]->configures >= 2;}, 10s));
  EXPECT_EQ(state("planner"), State::PRIMARY_STATE_UNCONFIGURED);
  kill("domain_expert");
  add("domain_expert");
  EXPECT_TRUE(becomes("domain_expert", State::PRIMARY_STATE_ACTIVE));
}

// All of PlanSys2 started again at once (or a startup that failed): each one is brought up,
// the executor last
TEST_F(SupervisorTest, everything_started_again_is_brought_up)
{
  for (const auto & name : kNames) {
    kill(name);
  }
  for (const auto & name : kNames) {
    add(name);
  }
  for (const auto & name : kNames) {
    EXPECT_TRUE(becomes(name, State::PRIMARY_STATE_ACTIVE, 10s)) << name;
  }
  EXPECT_FALSE(supervisor_->degraded());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
