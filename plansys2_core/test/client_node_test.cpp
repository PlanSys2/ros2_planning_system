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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/TestUtils.hpp"

using namespace std::chrono_literals;
using GetState = lifecycle_msgs::srv::GetState;

namespace
{

// A GetState server on a node of its own, spun in its own thread, answering after a delay
class Server
{
public:
  explicit Server(const std::string & name, std::chrono::milliseconds delay = 0ms)
  : node_(rclcpp::Node::make_shared(name))
  {
    service_ = node_->create_service<GetState>(
      name + "/get_state",
      [this, delay](const GetState::Request::SharedPtr, GetState::Response::SharedPtr response) {
        calls++;
        std::this_thread::sleep_for(delay);
        response->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      });
    exe_.add_node(node_);
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  }

  std::atomic<int> calls {0};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<GetState>::SharedPtr service_;
  rclcpp::executors::SingleThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

// A node spun in a thread of its own, as an application may spin the node of its clients
template<typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class Spun
{
public:
  template<typename NodeT>
  explicit Spun(const std::shared_ptr<NodeT> & node)
  {
    exe_.add_node(node->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  }

private:
  ExecutorT exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

// One call through the ClientNode: whether it got the answer in time
bool call(
  plansys2::ClientNode & client_node, const rclcpp::Client<GetState>::SharedPtr & client,
  std::chrono::seconds timeout = 5s)
{
  if (!client->wait_for_service(timeout)) {
    return false;
  }
  auto future = client->async_send_request(std::make_shared<GetState::Request>());
  if (client_node.wait(future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    client->remove_pending_request(future);
    return false;
  }
  return future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

}  // namespace

TEST(client_node, creates_no_node)
{
  Server server("no_node_server");
  auto node = rclcpp::Node::make_shared("no_node_host");
  Spun<> spun(node);
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return node->get_node_names().size() >= 2;}, 5s));
  auto before = node->get_node_names();

  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("no_node_server/get_state");
  auto sub = client_node.create_subscription<std_msgs::msg::String>(
    "no_node_topic", 10, [](std_msgs::msg::String::SharedPtr) {});
  ASSERT_TRUE(call(client_node, client));

  // Still the same nodes, given some time for one that would appear
  std::this_thread::sleep_for(500ms);
  auto after = node->get_node_names();
  std::sort(before.begin(), before.end());
  std::sort(after.begin(), after.end());
  EXPECT_EQ(before, after);
  EXPECT_STREQ(client->get_service_name(), "/no_node_server/get_state");
}

TEST(client_node, does_not_own_the_node)
{
  auto node = rclcpp::Node::make_shared("not_owned_host");
  auto base = node->get_node_base_interface();
  const auto node_uses = node.use_count();
  const auto base_uses = base.use_count();
  {
    plansys2::ClientNode client_node(*node);
    // Only the copies of lock() hold the node, while they live
    EXPECT_EQ(node.use_count(), node_uses);
    EXPECT_EQ(base.use_count(), base_uses);
    EXPECT_TRUE(client_node.lock().has_value());
  }
  EXPECT_EQ(base.use_count(), base_uses);
}

TEST(client_node, works_on_a_node_and_on_a_lifecycle_node)
{
  Server server("both_server");
  auto node = rclcpp::Node::make_shared("both_host");
  auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("both_lifecycle_host");

  plansys2::ClientNode on_node(*node);
  plansys2::ClientNode on_lifecycle_node(*lifecycle_node);
  Spun<> spun_node(node);
  Spun<> spun_lifecycle_node(lifecycle_node);
  EXPECT_EQ(on_node.get_name(), "both_host");
  EXPECT_EQ(on_lifecycle_node.get_name(), "both_lifecycle_host");
  EXPECT_EQ(on_node.get_namespace(), "/");

  ASSERT_TRUE(call(on_node, on_node.create_client<GetState>("both_server/get_state")));
  ASSERT_TRUE(
    call(on_lifecycle_node, on_lifecycle_node.create_client<GetState>("both_server/get_state")));
  EXPECT_EQ(server.calls.load(), 2);
}

TEST(client_node, names_follow_the_namespace_of_the_node)
{
  auto node = rclcpp::Node::make_shared("ns_host", "robot1");
  plansys2::ClientNode client_node(*node);
  EXPECT_EQ(client_node.get_namespace(), "/robot1");
  auto client = client_node.create_client<GetState>("domain_expert/get_state");
  EXPECT_STREQ(client->get_service_name(), "/robot1/domain_expert/get_state");
}

TEST(client_node, a_node_nobody_spins_gets_its_answer)
{
  // The client serves its group while it waits: the application need not spin its node
  Server server("unspun_server");
  auto node = rclcpp::Node::make_shared("unspun_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("unspun_server/get_state");
  EXPECT_TRUE(call(client_node, client));
  EXPECT_TRUE(call(client_node, client));
  EXPECT_EQ(server.calls.load(), 2);

  // Nor does it change once the node is spun
  Spun<> spun(node);
  EXPECT_TRUE(call(client_node, client));
}

TEST(client_node, calls_from_a_callback_with_a_multithreaded_executor)
{
  Server server("callback_server");
  auto node = rclcpp::Node::make_shared("callback_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("callback_server/get_state");

  std::promise<bool> answered;
  std::atomic<bool> fired {false};
  auto timer = node->create_wall_timer(
    50ms, [&]() {
      if (!fired.exchange(true)) {
        answered.set_value(call(client_node, client));
      }
    });
  Spun<rclcpp::executors::MultiThreadedExecutor> spun(node);

  auto result = answered.get_future();
  ASSERT_EQ(result.wait_for(10s), std::future_status::ready);
  EXPECT_TRUE(result.get());
}

TEST(client_node, calls_from_a_callback_with_a_single_threaded_executor)
{
  // The only thread of the executor is in the callback: the client brings its answer itself
  Server server("single_server");
  auto node = rclcpp::Node::make_shared("single_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("single_server/get_state");

  std::promise<bool> answered;
  std::atomic<int> fired {0};
  auto timer = node->create_wall_timer(
    50ms, [&]() {
      // Several calls in a row from the same callback
      if (fired++ == 0) {
        answered.set_value(call(client_node, client) && call(client_node, client));
      }
    });
  Spun<> spun(node);

  auto result = answered.get_future();
  ASSERT_EQ(result.wait_for(10s), std::future_status::ready);
  EXPECT_TRUE(result.get());
  // The timer goes on: the executor of the node was not blocked for long
  EXPECT_TRUE(plansys2::testing::wait_for([&]() {return fired.load() > 3;}, 5s));
}

TEST(client_node, the_executor_of_the_node_does_not_take_its_group)
{
  // A callback of the client runs only while the client serves its group
  Server server("group_server");
  auto node = rclcpp::Node::make_shared("group_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("group_server/get_state");
  ASSERT_TRUE(client->wait_for_service(5s));

  std::atomic<int> responses {0};
  Spun<> spun(node);
  auto future = client->async_send_request(
    std::make_shared<GetState::Request>(),
    [&](rclcpp::Client<GetState>::SharedFuture) {responses++;});
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(responses.load(), 0);
  EXPECT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        client_node.serve();
        return responses.load() == 1;
      }, 5s));
}

TEST(client_node, subscriptions_come_when_the_client_serves)
{
  auto node = rclcpp::Node::make_shared("sub_host");
  plansys2::ClientNode client_node(*node);
  std::atomic<int> received {0};
  auto sub = client_node.create_subscription<std_msgs::msg::String>(
    "sub_topic", rclcpp::QoS(10).reliable(), [&](std_msgs::msg::String::SharedPtr) {received++;});

  auto publisher_node = rclcpp::Node::make_shared("sub_publisher");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
    "sub_topic", rclcpp::QoS(10).reliable());
  ASSERT_TRUE(
    plansys2::testing::wait_for([&]() {return publisher->get_subscription_count() > 0;}, 5s));

  // Spun or not, the node does not take them: serve() brings all that came
  Spun<> spun(node);
  for (int i = 0; i < 3; i++) {
    publisher->publish(std_msgs::msg::String());
  }
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(received.load(), 0);
  EXPECT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        client_node.serve();
        return received.load() == 3;
      }, 5s));
  client_node.serve();
  EXPECT_EQ(received.load(), 3);
}

TEST(client_node, a_controller_with_its_own_subscription_spun_by_one_thread)
{
  // As a controller: its subscription for the executor of the node, a timer that calls
  Server server("controller_server");
  auto node = rclcpp::Node::make_shared("controller_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("controller_server/get_state");

  std::atomic<int> received {0};
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "controller_topic", rclcpp::QoS(10).reliable(),
    [&](std_msgs::msg::String::SharedPtr) {received++;});
  std::atomic<int> answered {0};
  std::atomic<int> failed {0};
  auto timer = node->create_wall_timer(
    50ms, [&]() {
      if (call(client_node, client)) {
        answered++;
      } else {
        failed++;
      }
    });

  auto publisher_node = rclcpp::Node::make_shared("controller_publisher");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
    "controller_topic", rclcpp::QoS(10).reliable());
  ASSERT_TRUE(
    plansys2::testing::wait_for([&]() {return publisher->get_subscription_count() > 0;}, 5s));

  Spun<> spun(node);
  for (int i = 0; i < 5; i++) {
    publisher->publish(std_msgs::msg::String());
    std::this_thread::sleep_for(50ms);
  }
  // Both go on: the messages of the controller and the answers of its calls
  EXPECT_TRUE(plansys2::testing::wait_for([&]() {return received.load() == 5;}, 5s));
  EXPECT_TRUE(plansys2::testing::wait_for([&]() {return answered.load() >= 5;}, 5s));
  EXPECT_EQ(failed.load(), 0);
}

TEST(client_node, concurrent_calls_on_a_node_nobody_spins)
{
  // One thread serves the group, the others see their answers come
  Server server("unspun_concurrent_server", 20ms);
  auto node = rclcpp::Node::make_shared("unspun_concurrent_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("unspun_concurrent_server/get_state");
  ASSERT_TRUE(client->wait_for_service(5s));

  constexpr int kThreads = 4;
  constexpr int kCalls = 10;
  std::atomic<int> answered {0};
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; t++) {
    threads.emplace_back(
      [&]() {
        for (int i = 0; i < kCalls; i++) {
          if (call(client_node, client)) {
            answered++;
          }
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(answered.load(), kThreads * kCalls);
}

TEST(client_node, concurrent_calls_from_several_threads)
{
  Server server("concurrent_server", 20ms);
  auto node = rclcpp::Node::make_shared("concurrent_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("concurrent_server/get_state");
  ASSERT_TRUE(client->wait_for_service(5s));
  Spun<> spun(node);

  constexpr int kThreads = 4;
  constexpr int kCalls = 10;
  std::atomic<int> answered {0};
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; t++) {
    threads.emplace_back(
      [&]() {
        for (int i = 0; i < kCalls; i++) {
          if (call(client_node, client)) {
            answered++;
          }
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(answered.load(), kThreads * kCalls);
  EXPECT_EQ(server.calls.load(), kThreads * kCalls);
}

TEST(client_node, several_on_the_same_node)
{
  Server server("several_server");
  auto node = rclcpp::Node::make_shared("several_host");
  plansys2::ClientNode first(*node);
  plansys2::ClientNode second(*node);
  EXPECT_NE(first.group(), second.group());
  Spun<> spun(node);

  auto client_first = first.create_client<GetState>("several_server/get_state");
  auto client_second = second.create_client<GetState>("several_server/get_state");
  std::atomic<bool> ok_first {false};
  std::thread other([&]() {ok_first = call(first, client_first);});
  EXPECT_TRUE(call(second, client_second));
  other.join();
  EXPECT_TRUE(ok_first.load());
}

TEST(client_node, a_slow_answer_ends_at_the_timeout)
{
  Server server("slow_server", 2000ms);
  auto node = rclcpp::Node::make_shared("slow_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("slow_server/get_state");
  ASSERT_TRUE(client->wait_for_service(5s));
  Spun<> spun(node);

  auto future = client->async_send_request(std::make_shared<GetState::Request>());
  auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(client_node.wait(future, 300ms), rclcpp::FutureReturnCode::TIMEOUT);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1500ms);
  // A later wait still gets it
  EXPECT_EQ(client_node.wait(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
}

TEST(client_node, after_the_node_is_gone)
{
  auto node = rclcpp::Node::make_shared("gone_host");
  auto client_node = std::make_shared<plansys2::ClientNode>(*node);
  auto client = client_node->create_client<GetState>("nobody/get_state");
  node.reset();

  EXPECT_FALSE(client_node->lock().has_value());
  EXPECT_THROW(client_node->create_client<GetState>("nobody/get_state"), std::runtime_error);
  EXPECT_THROW(
    client_node->create_subscription<std_msgs::msg::String>(
      "gone_topic", 10, [](std_msgs::msg::String::SharedPtr) {}), std::runtime_error);
  // What it kept still answers without the node
  EXPECT_EQ(client_node->get_name(), "gone_host");
  EXPECT_NO_THROW(client_node->now());
  EXPECT_NO_THROW(client_node->serve());
  auto future = client->async_send_request(std::make_shared<GetState::Request>());
  EXPECT_EQ(client_node->wait(future, 200ms), rclcpp::FutureReturnCode::TIMEOUT);
  client_node.reset();
}

TEST(client_node, destroyed_before_the_node)
{
  Server server("before_server");
  auto node = rclcpp::Node::make_shared("before_host");
  Spun<> spun(node);
  {
    plansys2::ClientNode client_node(*node);
    ASSERT_TRUE(call(client_node, client_node.create_client<GetState>("before_server/get_state")));
  }
  // The node goes on, and another one works on it
  plansys2::ClientNode again(*node);
  EXPECT_TRUE(call(again, again.create_client<GetState>("before_server/get_state")));
}

// The last one: it shuts ROS down
TEST(client_node, a_shutdown_ends_the_wait)
{
  Server server("shutdown_server", 5000ms);
  auto node = rclcpp::Node::make_shared("shutdown_host");
  plansys2::ClientNode client_node(*node);
  auto client = client_node.create_client<GetState>("shutdown_server/get_state");
  ASSERT_TRUE(client->wait_for_service(5s));
  Spun<> spun(node);

  auto future = client->async_send_request(std::make_shared<GetState::Request>());
  std::thread stopper([]() {
      std::this_thread::sleep_for(300ms);
      rclcpp::shutdown();
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(client_node.wait(future, 30s), rclcpp::FutureReturnCode::INTERRUPTED);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
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
