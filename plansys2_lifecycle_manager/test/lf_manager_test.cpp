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

#include <memory>
#include <string>
#include <map>
#include <vector>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

#include "fake_lifecycle_node.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using ClientMap = std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>>;

namespace
{

const std::vector<std::string> kNodes = {"planner", "domain_expert", "problem_expert", "executor"};

// Clients for the four PlanSys2 keys, managing the nodes prefix + key, on one manager node
ClientMap make_clients(const std::string & prefix, Spinner & spinner)
{
  auto manager = rclcpp::Node::make_shared(prefix + "lifecycle_manager");
  spinner.add(manager);  // Keeps it alive: the clients do not own it
  auto client_node = std::make_shared<plansys2::ClientNode>(*manager);
  ClientMap clients;
  for (const auto & key : kNodes) {
    clients[key] = std::make_shared<plansys2::LifecycleServiceClient>(client_node, prefix + key);
  }
  return clients;
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

TEST(lifecycle_manager, lf_client)
{
  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test");
  auto manager = rclcpp::Node::make_shared("mng_client");
  plansys2::testing::NodeSpinner manager_spinner(manager);
  auto client_node = std::make_shared<plansys2::LifecycleServiceClient>(
    std::make_shared<plansys2::ClientNode>(*manager), "test");

  // Only the managed node is spun: the client serves its answers while it waits
  auto exe = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exe->add_node(test_node->get_node_base_interface());

  plansys2::testing::ExecutorThread spinner(*exe);

  // The client must see both lifecycle services of the node
  ASSERT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        return plansys2::testing::has_service(*manager, "test/get_state") &&
               plansys2::testing::has_service(*manager, "test/change_state");
      }));

  ASSERT_EQ(
    test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  ASSERT_FALSE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST(lifecycle_manager, lf_startup)
{
  auto de_node = rclcpp_lifecycle::LifecycleNode::make_shared("domain_expert");
  auto pe_node = rclcpp_lifecycle::LifecycleNode::make_shared("problem_expert");
  auto pl_node = rclcpp_lifecycle::LifecycleNode::make_shared("planner");
  auto ex_node = rclcpp_lifecycle::LifecycleNode::make_shared("executor");

  ASSERT_EQ(
    de_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    pe_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    pl_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    ex_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto manager = rclcpp::Node::make_shared("lifecycle_manager");
  plansys2::testing::NodeSpinner manager_spinner(manager);
  auto client_node = std::make_shared<plansys2::ClientNode>(*manager);
  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  for (const auto & key : kNodes) {
    manager_nodes[key] = std::make_shared<plansys2::LifecycleServiceClient>(client_node, key);
  }

  rclcpp::executors::MultiThreadedExecutor exe;

  exe.add_node(de_node->get_node_base_interface());
  exe.add_node(pe_node->get_node_base_interface());
  exe.add_node(pl_node->get_node_base_interface());
  exe.add_node(ex_node->get_node_base_interface());

  plansys2::testing::ExecutorThread spinner(exe);

  // Each manager must see the lifecycle services of its node
  for (const auto & name : kNodes) {
    ASSERT_TRUE(
      plansys2::testing::wait_for(
        [&]() {
          return plansys2::testing::has_service(*manager, name + "/get_state") &&
                 plansys2::testing::has_service(*manager, name + "/change_state");
        })) << name;
  }

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(3)));

  startup_future.wait();

  ASSERT_EQ(
    de_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    pe_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    pl_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    ex_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST(lifecycle_manager, client_without_node)
{
  auto manager = rclcpp::Node::make_shared("nobody_client");
  plansys2::testing::NodeSpinner manager_spinner(manager);
  auto client = std::make_shared<plansys2::LifecycleServiceClient>(
    std::make_shared<plansys2::ClientNode>(*manager), "nobody");

  auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(client->get_state(1s), State::PRIMARY_STATE_UNKNOWN);
  ASSERT_FALSE(client->change_state(Transition::TRANSITION_CONFIGURE, 1s));
  ASSERT_LT(seconds_since(start), 4.0);
}

TEST(lifecycle_manager, client_after_its_node_is_gone)
{
  // The client does not own the node: once it is gone, the calls fail fast, without throwing
  auto manager = rclcpp::Node::make_shared("gone_client");
  auto client = std::make_shared<plansys2::LifecycleServiceClient>(
    std::make_shared<plansys2::ClientNode>(*manager), "gone");
  manager.reset();

  auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(client->get_state(1s), State::PRIMARY_STATE_UNKNOWN);
  ASSERT_FALSE(client->change_state(Transition::TRANSITION_CONFIGURE, 1s));
  ASSERT_LT(seconds_since(start), 2.5);
}

TEST(lifecycle_manager, client_with_slow_node)
{
  // Answers after 3 s: the client must give up at its timeout, not wait for the answer
  Spinner fake_spinner;
  auto fake = std::make_shared<FakeLifecycleNode>("slow");
  fake->state = State::PRIMARY_STATE_INACTIVE;
  fake->get_state_delay = 3s;
  fake->change_state_delay = 3s;
  fake_spinner.add(fake);
  fake_spinner.start();

  auto manager = rclcpp::Node::make_shared("slow_client");
  plansys2::testing::NodeSpinner manager_spinner(manager);
  auto client = std::make_shared<plansys2::LifecycleServiceClient>(
    std::make_shared<plansys2::ClientNode>(*manager), "slow");

  auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(client->get_state(1s), State::PRIMARY_STATE_UNKNOWN);
  ASSERT_LT(seconds_since(start), 2.5);

  start = std::chrono::steady_clock::now();
  ASSERT_FALSE(client->change_state(Transition::TRANSITION_CONFIGURE, 1s));
  ASSERT_LT(seconds_since(start), 2.5);

  // Once it answers fast again, late answers do not get mixed up with new requests
  fake->get_state_delay = 0ms;
  // Kept: the late answers come when the fake ends its delays, with nothing to observe
  std::this_thread::sleep_for(4s);
  ASSERT_EQ(client->get_state(1s), State::PRIMARY_STATE_INACTIVE);
  fake->state = State::PRIMARY_STATE_ACTIVE;
  ASSERT_EQ(client->get_state(1s), State::PRIMARY_STATE_ACTIVE);
}

TEST(lifecycle_manager, startup_missing_client)
{
  Spinner spinner;
  auto clients = make_clients("missing_", spinner);
  auto planner = rclcpp_lifecycle::LifecycleNode::make_shared("missing_planner");
  spinner.add(planner);
  spinner.start();

  auto without_executor = clients;
  without_executor.erase("executor");
  ASSERT_FALSE(plansys2::startup_function(without_executor, 1s));
  ASSERT_EQ(without_executor.size(), 3u);
  // Checked before touching any node
  ASSERT_EQ(planner->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  auto null_problem_expert = clients;
  null_problem_expert["problem_expert"] = nullptr;
  ASSERT_FALSE(plansys2::startup_function(null_problem_expert, 1s));
  ASSERT_EQ(planner->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  ClientMap empty;
  ASSERT_FALSE(plansys2::startup_function(empty, 1s));
}

TEST(lifecycle_manager, startup_configure_fails)
{
  Spinner spinner;
  auto clients = make_clients("cfgfail_", spinner);
  auto planner = rclcpp_lifecycle::LifecycleNode::make_shared("cfgfail_planner");
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("cfgfail_domain_expert");
  auto problem = std::make_shared<FailingLifecycleNode>("cfgfail_problem_expert", true, false);
  auto executor = rclcpp_lifecycle::LifecycleNode::make_shared("cfgfail_executor");
  spinner.add(planner);
  spinner.add(domain);
  spinner.add(problem);
  spinner.add(executor);
  spinner.start();

  auto start = std::chrono::steady_clock::now();
  ASSERT_FALSE(plansys2::startup_function(clients, 2s));
  ASSERT_LT(seconds_since(start), 10.0);

  ASSERT_EQ(planner->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(domain->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(problem->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  // Stops at the first failure
  ASSERT_EQ(executor->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(lifecycle_manager, startup_activate_fails)
{
  Spinner spinner;
  auto clients = make_clients("actfail_", spinner);
  auto planner = rclcpp_lifecycle::LifecycleNode::make_shared("actfail_planner");
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("actfail_domain_expert");
  auto problem = rclcpp_lifecycle::LifecycleNode::make_shared("actfail_problem_expert");
  auto executor = std::make_shared<FailingLifecycleNode>("actfail_executor", false, true);
  spinner.add(planner);
  spinner.add(domain);
  spinner.add(problem);
  spinner.add(executor);
  spinner.start();

  ASSERT_FALSE(plansys2::startup_function(clients, 2s));
  ASSERT_EQ(domain->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(problem->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(planner->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(executor->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST(lifecycle_manager, startup_node_not_running)
{
  Spinner spinner;
  auto clients = make_clients("absent_", spinner);
  auto planner = rclcpp_lifecycle::LifecycleNode::make_shared("absent_planner");
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("absent_domain_expert");
  auto problem = rclcpp_lifecycle::LifecycleNode::make_shared("absent_problem_expert");
  // No executor node at all
  spinner.add(planner);
  spinner.add(domain);
  spinner.add(problem);
  spinner.start();

  auto start = std::chrono::steady_clock::now();
  ASSERT_FALSE(plansys2::startup_function(clients, 1s));
  ASSERT_LT(seconds_since(start), 6.0);
  ASSERT_EQ(problem->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST(lifecycle_manager, startup_state_never_reached)
{
  // The planner accepts the transition but always reports unconfigured: it used to loop forever
  Spinner spinner;
  auto clients = make_clients("stuck_", spinner);
  auto fake_planner = std::make_shared<FakeLifecycleNode>("stuck_planner");
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("stuck_domain_expert");
  spinner.add(fake_planner);
  spinner.add(domain);
  spinner.start();

  auto start = std::chrono::steady_clock::now();
  ASSERT_FALSE(plansys2::startup_function(clients, 1s));
  ASSERT_LT(seconds_since(start), 5.0);
  ASSERT_GT(fake_planner->get_state_calls.load(), 1);
  ASSERT_EQ(domain->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(lifecycle_manager, startup_state_reached_late)
{
  // The planner reports inactive a bit after accepting the transition: startup waits for it
  Spinner spinner;
  auto clients = make_clients("late_", spinner);
  auto fake_planner = std::make_shared<FakeLifecycleNode>("late_planner");
  std::weak_ptr<FakeLifecycleNode> weak_fake = fake_planner;
  fake_planner->on_change_state = [weak_fake]() {
      std::thread([weak_fake]() {
          std::this_thread::sleep_for(500ms);
          if (auto fake = weak_fake.lock()) {
            fake->state = fake->state == State::PRIMARY_STATE_UNCONFIGURED ?
            State::PRIMARY_STATE_INACTIVE : State::PRIMARY_STATE_ACTIVE;
          }
        }).detach();
    };
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("late_domain_expert");
  auto problem = rclcpp_lifecycle::LifecycleNode::make_shared("late_problem_expert");
  auto executor = rclcpp_lifecycle::LifecycleNode::make_shared("late_executor");
  spinner.add(fake_planner);
  spinner.add(domain);
  spinner.add(problem);
  spinner.add(executor);
  spinner.start();

  ASSERT_TRUE(plansys2::startup_function(clients, 3s));
  ASSERT_EQ(fake_planner->state.load(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(executor->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST(lifecycle_manager, startup_with_lost_responses)
{
  // The planner makes each transition but its response is lost: startup sees its state
  Spinner spinner;
  auto clients = make_clients("lost_", spinner);
  auto fake_planner = std::make_shared<FakeLifecycleNode>("lost_planner");
  fake_planner->lose_change_state_response = true;
  std::weak_ptr<FakeLifecycleNode> weak_fake = fake_planner;
  fake_planner->on_change_state = [weak_fake]() {
      if (auto fake = weak_fake.lock()) {
        fake->state = fake->state == State::PRIMARY_STATE_UNCONFIGURED ?
          State::PRIMARY_STATE_INACTIVE : State::PRIMARY_STATE_ACTIVE;
      }
    };
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("lost_domain_expert");
  auto problem = rclcpp_lifecycle::LifecycleNode::make_shared("lost_problem_expert");
  auto executor = rclcpp_lifecycle::LifecycleNode::make_shared("lost_executor");
  spinner.add(fake_planner);
  spinner.add(domain);
  spinner.add(problem);
  spinner.add(executor);
  spinner.start();

  ASSERT_TRUE(plansys2::startup_function(clients, 1s));
  ASSERT_EQ(fake_planner->state.load(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(executor->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST(lifecycle_manager, startup_with_lost_response_of_a_transition_not_made)
{
  // No response and no change: it is a failure, after waiting for the state
  Spinner spinner;
  auto clients = make_clients("lost_failed_", spinner);
  auto fake_planner = std::make_shared<FakeLifecycleNode>("lost_failed_planner");
  fake_planner->lose_change_state_response = true;
  auto domain = rclcpp_lifecycle::LifecycleNode::make_shared("lost_failed_domain_expert");
  spinner.add(fake_planner);
  spinner.add(domain);
  spinner.start();

  auto start = std::chrono::steady_clock::now();
  ASSERT_FALSE(plansys2::startup_function(clients, 1s));
  ASSERT_LT(seconds_since(start), 5.0);
  ASSERT_EQ(domain->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  // Before the process exits: Zenoh aborts if ROS is shut down by atexit
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
