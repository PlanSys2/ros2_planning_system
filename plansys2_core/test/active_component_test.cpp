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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_msgs/srv/get_domain_name.hpp"
#include "rclcpp/rclcpp.hpp"

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_core/TestUtils.hpp"

using namespace std::chrono_literals;
using GetState = lifecycle_msgs::srv::GetState;
using lifecycle_msgs::msg::State;

namespace
{

// A component: its lifecycle state, as the test sets it, and a service of its own (ping)
class FakeComponent
{
public:
  FakeComponent(const std::string & component, uint8_t initial)
  : state(initial), node_(rclcpp::Node::make_shared("fake_" + component))
  {
    get_state_ = node_->create_service<GetState>(
      component + "/get_state",
      [this](const GetState::Request::SharedPtr, GetState::Response::SharedPtr response) {
        state_requests++;
        response->current_state.id = state;
      });
    ping_ = node_->create_service<GetState>(
      component + "/ping",
      [this](const GetState::Request::SharedPtr, GetState::Response::SharedPtr response) {
        pings++;
        response->current_state.id = State::PRIMARY_STATE_ACTIVE;
      });
    spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(node_);
  }

  std::atomic<uint8_t> state;
  std::atomic<int> state_requests {0};
  std::atomic<int> pings {0};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<GetState>::SharedPtr get_state_;
  rclcpp::Service<GetState>::SharedPtr ping_;
  std::unique_ptr<plansys2::testing::NodeSpinner> spinner_;
};

// What a PlanSys2 client has: its node, spun by the application, and its component
struct Client
{
  explicit Client(const std::string & component)
  : node(rclcpp::Node::make_shared(component + "_client")),
    client_node(std::make_shared<plansys2::ClientNode>(*node)),
    spinner(node),
    active(std::make_shared<plansys2::ActiveComponent>(client_node, component)),
    ping(client_node, component + "/ping", active)
  {
  }

  bool call() {return ping.call(std::make_shared<GetState::Request>()) != nullptr;}

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<plansys2::ClientNode> client_node;
  plansys2::testing::NodeSpinner spinner;
  std::shared_ptr<plansys2::ActiveComponent> active;
  plansys2::ServiceClient<GetState> ping;
};

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

TEST(active_component, a_call_waits_for_a_component_that_starts_late)
{
  // The application first; the component some seconds later, already active
  Client client("late");
  std::unique_ptr<FakeComponent> component;
  std::thread starter([&component]() {
      std::this_thread::sleep_for(2500ms);
      component = std::make_unique<FakeComponent>("late", State::PRIMARY_STATE_ACTIVE);
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client.call());
  EXPECT_GE(seconds_since(start), 2.4);
  starter.join();
  EXPECT_EQ(component->pings.load(), 1);
}

TEST(active_component, a_call_waits_until_the_component_is_active)
{
  // There, with its services, but not active yet: nothing is sent to it before
  FakeComponent component("inactive", State::PRIMARY_STATE_INACTIVE);
  Client client("inactive");
  std::thread activator([&component]() {
      std::this_thread::sleep_for(2s);
      component.state = State::PRIMARY_STATE_ACTIVE;
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client.call());
  EXPECT_GE(seconds_since(start), 1.9);
  activator.join();
  EXPECT_EQ(component.pings.load(), 1);
  EXPECT_GE(component.state_requests.load(), 2);
}

TEST(active_component, seen_active_it_is_not_asked_again)
{
  FakeComponent component("seen", State::PRIMARY_STATE_ACTIVE);
  Client client("seen");
  ASSERT_TRUE(client.call());
  const int asked = component.state_requests.load();
  for (int i = 0; i < 10; i++) {
    ASSERT_TRUE(client.call());
  }
  EXPECT_EQ(component.state_requests.load(), asked);
  EXPECT_EQ(component.pings.load(), 11);
}

TEST(active_component, a_restarted_component_is_waited_for_again)
{
  Client client("restarted");
  auto component = std::make_unique<FakeComponent>("restarted", State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(client.call());

  // It stops; it comes back after a while, active
  component.reset();
  std::thread restarter([&component]() {
      std::this_thread::sleep_for(3s);
      component = std::make_unique<FakeComponent>("restarted", State::PRIMARY_STATE_ACTIVE);
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client.call());
  EXPECT_GE(seconds_since(start), 2.5);
  restarter.join();
  EXPECT_EQ(component->pings.load(), 1);
}

TEST(active_component, wait_alone_is_the_same_wait)
{
  FakeComponent component("alone", State::PRIMARY_STATE_INACTIVE);
  Client client("alone");
  std::thread activator([&component]() {
      std::this_thread::sleep_for(1500ms);
      component.state = State::PRIMARY_STATE_ACTIVE;
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client.active->wait());
  EXPECT_GE(seconds_since(start), 1.4);
  activator.join();
  // Already active: at once
  start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client.active->wait());
  EXPECT_LT(seconds_since(start), 0.1);
}

TEST(active_component, concurrent_calls_wait_together)
{
  FakeComponent component("together", State::PRIMARY_STATE_INACTIVE);
  Client client("together");
  std::atomic<int> answered {0};
  std::vector<std::thread> callers;
  for (int i = 0; i < 4; i++) {
    callers.emplace_back([&]() {if (client.call()) {answered++;}});
  }
  std::this_thread::sleep_for(1500ms);
  EXPECT_EQ(component.pings.load(), 0);
  component.state = State::PRIMARY_STATE_ACTIVE;
  for (auto & caller : callers) {
    caller.join();
  }
  EXPECT_EQ(answered.load(), 4);
  EXPECT_EQ(component.pings.load(), 4);
}

TEST(active_component, an_answer_saying_not_active_waits_and_asks_again)
{
  // Seen active, then deactivated: its service answers kNotActive, the call waits until it is
  // active again and asks again
  FakeComponent component("deactivated", State::PRIMARY_STATE_ACTIVE);
  auto node = rclcpp::Node::make_shared("deactivated_server");
  std::atomic<int> names {0};
  auto service = node->create_service<plansys2_msgs::srv::GetDomainName>(
    "deactivated/name",
    [&](const plansys2_msgs::srv::GetDomainName::Request::SharedPtr,
    plansys2_msgs::srv::GetDomainName::Response::SharedPtr response) {
      names++;
      if (component.state != State::PRIMARY_STATE_ACTIVE) {
        response->success = false;
        response->error_info = plansys2::kNotActive;
      } else {
        response->success = true;
        response->name = "domain";
      }
    });
  plansys2::testing::NodeSpinner server_spinner(node);

  Client client("deactivated");
  plansys2::ServiceClient<plansys2_msgs::srv::GetDomainName> name(
    client.client_node, "deactivated/name", client.active);
  ASSERT_TRUE(client.call());  // Seen active

  component.state = State::PRIMARY_STATE_INACTIVE;
  std::thread activator([&component]() {
      std::this_thread::sleep_for(2s);
      component.state = State::PRIMARY_STATE_ACTIVE;
    });
  auto start = std::chrono::steady_clock::now();
  auto response = name.call(std::make_shared<plansys2_msgs::srv::GetDomainName::Request>());
  activator.join();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->name, "domain");
  EXPECT_GE(seconds_since(start), 1.9);
  EXPECT_EQ(names.load(), 2);
}

TEST(active_component, a_component_that_always_says_not_active_is_an_error)
{
  // Active by its lifecycle, yet its service says it is not: after some waits, the answer
  FakeComponent component("liar", State::PRIMARY_STATE_ACTIVE);
  auto node = rclcpp::Node::make_shared("liar_server");
  std::atomic<int> names {0};
  auto service = node->create_service<plansys2_msgs::srv::GetDomainName>(
    "liar/name",
    [&](const plansys2_msgs::srv::GetDomainName::Request::SharedPtr,
    plansys2_msgs::srv::GetDomainName::Response::SharedPtr response) {
      names++;
      response->success = false;
      response->error_info = plansys2::kNotActive;
    });
  plansys2::testing::NodeSpinner server_spinner(node);

  Client client("liar");
  plansys2::ServiceClient<plansys2_msgs::srv::GetDomainName> name(
    client.client_node, "liar/name", client.active);
  auto response = name.call(std::make_shared<plansys2_msgs::srv::GetDomainName::Request>());
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->error_info, plansys2::kNotActive);
  EXPECT_EQ(names.load(), 4);
}

// The last one: it shuts ROS down
// With a max wait, a component that never gets active ends the wait: a plan does not wait for
// ever for one that is gone
TEST(active_component, a_max_wait_gives_up)
{
  Client client("never");
  FakeComponent component("never", State::PRIMARY_STATE_INACTIVE);
  client.active->set_max_wait(1500ms);
  auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.call());
  EXPECT_GT(seconds_since(start), 1.4);
  EXPECT_LT(seconds_since(start), 4.0);
  // Nor is there any component at all
  Client nobody("nobody");
  nobody.active->set_max_wait(1s);
  start = std::chrono::steady_clock::now();
  EXPECT_FALSE(nobody.call());
  EXPECT_LT(seconds_since(start), 4.0);
}

// The max wait is for each wait: one that gets active in time goes on
TEST(active_component, a_max_wait_lets_in_time_ones_go_on)
{
  Client client("in_time");
  client.active->set_max_wait(5s);
  std::unique_ptr<FakeComponent> component;
  std::thread starter([&component]() {
      std::this_thread::sleep_for(1500ms);
      component = std::make_unique<FakeComponent>("in_time", State::PRIMARY_STATE_ACTIVE);
    });
  EXPECT_TRUE(client.call());
  starter.join();
  EXPECT_TRUE(client.call());
}

TEST(active_component, a_shutdown_ends_the_wait)
{
  Client client("never");
  std::thread stopper([]() {
      std::this_thread::sleep_for(1500ms);
      rclcpp::shutdown();
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.call());
  EXPECT_LT(seconds_since(start), 5.0);
  stopper.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
