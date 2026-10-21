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
#include "rclcpp/rclcpp.hpp"

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_core/TestUtils.hpp"

using namespace std::chrono_literals;
using GetState = lifecycle_msgs::srv::GetState;

namespace
{

// A GetState server that leaves the first `lost` requests without answer, as a lost response
class Server
{
public:
  explicit Server(const std::string & name, int lost = 0)
  : node_(rclcpp::Node::make_shared(name)), lost_(lost)
  {
    service_ = node_->create_service<GetState>(
      name + "/get_state",
      [this](
        std::shared_ptr<rmw_request_id_t> header, const GetState::Request::SharedPtr) {
        if (calls++ < lost_) {
          return;  // Never answered
        }
        GetState::Response response;
        response.current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        service_->send_response(*header, response);
      });
    spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(node_);
  }

  std::atomic<int> calls {0};

private:
  rclcpp::Node::SharedPtr node_;
  int lost_;
  rclcpp::Service<GetState>::SharedPtr service_;
  std::unique_ptr<plansys2::testing::NodeSpinner> spinner_;
};

// The node of the clients, spun as an application spins it
struct Host
{
  explicit Host(const std::string & name)
  : node(rclcpp::Node::make_shared(name)),
    client_node(std::make_shared<plansys2::ClientNode>(*node)),
    spinner(node)
  {
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<plansys2::ClientNode> client_node;
  plansys2::testing::NodeSpinner spinner;
};

bool active(const GetState::Response::SharedPtr & response)
{
  return response &&
         response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

TEST(service_client, answers)
{
  Server server("sc_answers");
  Host host("sc_answers_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_answers/get_state");
  EXPECT_EQ(client.name(), "/sc_answers/get_state");
  ASSERT_TRUE(client.wait_for_service(5s));
  EXPECT_TRUE(active(client.call(std::make_shared<GetState::Request>())));
  EXPECT_EQ(server.calls.load(), 1);
}

TEST(service_client, a_lost_answer_is_asked_again_with_a_new_client)
{
  Server server("sc_lost", 2);
  Host host("sc_lost_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_lost/get_state");
  ASSERT_TRUE(client.wait_for_service(5s));

  EXPECT_TRUE(active(client.call(std::make_shared<GetState::Request>(), 500ms, 3, 5s)));
  EXPECT_EQ(server.calls.load(), 3);
}

TEST(service_client, without_answer_it_gives_up_after_its_attempts)
{
  Server server("sc_silent", 1000);
  Host host("sc_silent_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_silent/get_state");
  ASSERT_TRUE(client.wait_for_service(5s));

  auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(client.call(std::make_shared<GetState::Request>(), 300ms, 3, 5s), nullptr);
  EXPECT_EQ(server.calls.load(), 3);
  // Three waits, and the discovery of each new client
  EXPECT_GE(seconds_since(start), 0.9);
  EXPECT_LT(seconds_since(start), 10.0);
}

TEST(service_client, one_attempt_sends_it_once_and_leaves_a_new_client)
{
  Server server("sc_once", 1);
  Host host("sc_once_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_once/get_state");
  ASSERT_TRUE(client.wait_for_service(5s));

  EXPECT_EQ(client.call(std::make_shared<GetState::Request>(), 300ms, 1, 5s), nullptr);
  EXPECT_EQ(server.calls.load(), 1);
  // The next call, on the new client, gets its answer
  EXPECT_TRUE(active(client.call(std::make_shared<GetState::Request>(), 1s, 1, 5s)));
  EXPECT_EQ(server.calls.load(), 2);
}

TEST(service_client, without_server_no_request)
{
  Host host("sc_nobody_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_nobody/get_state");
  auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(client.call(std::make_shared<GetState::Request>(), 1s, 3, 500ms), nullptr);
  // Only the wait for the server: no attempt without it
  EXPECT_LT(seconds_since(start), 2.0);
  EXPECT_FALSE(client.service_is_ready());
}

TEST(service_client, waits_for_a_server_that_comes_within_its_timeout)
{
  Host host("sc_late_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_late/get_state");
  std::unique_ptr<Server> server;
  std::thread starter([&server]() {
      std::this_thread::sleep_for(1500ms);
      server = std::make_unique<Server>("sc_late");
    });
  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(
    active(client.call(std::make_shared<GetState::Request>(), 1s, 3, 5s)));
  EXPECT_GE(seconds_since(start), 1.4);
  starter.join();
}

TEST(service_client, concurrent_calls_with_lost_answers)
{
  // Several threads lose answers at once: each renewal is made once, every call ends
  Server server("sc_concurrent", 4);
  Host host("sc_concurrent_host");
  plansys2::ServiceClient<GetState> client(host.client_node, "sc_concurrent/get_state");
  ASSERT_TRUE(client.wait_for_service(5s));

  std::atomic<int> answered {0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; t++) {
    threads.emplace_back(
      [&]() {
        for (int i = 0; i < 5; i++) {
          if (active(client.call(std::make_shared<GetState::Request>(), 500ms, 3, 5s))) {
            answered++;
          }
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  // The four lost requests are asked again: all the calls answer
  EXPECT_EQ(answered.load(), 20);
  EXPECT_EQ(server.calls.load(), 24);
}

TEST(service_client, after_its_node_is_gone)
{
  // No answer and no crash: the node of a new client is gone too
  Server server("sc_gone", 1000);
  auto node = rclcpp::Node::make_shared("sc_gone_host");
  auto client_node = std::make_shared<plansys2::ClientNode>(*node);
  plansys2::ServiceClient<GetState> client(client_node, "sc_gone/get_state");
  node.reset();
  EXPECT_EQ(client.call(std::make_shared<GetState::Request>(), 200ms, 3, 1s), nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
