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
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_msgs/srv/get_domain_name.hpp"
#include "rclcpp/rclcpp.hpp"

#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_core/TestUtils.hpp"

using GetDomainName = plansys2_msgs::srv::GetDomainName;
using GetState = lifecycle_msgs::srv::GetState;
using namespace std::chrono_literals;

// What a request from hostile input throws ends as a failed answer, and the node goes on
TEST(guarded_service, an_exception_is_a_failed_answer)
{
  auto node = rclcpp::Node::make_shared("guarded_host");
  std::atomic<int> calls {0};
  auto service = node->create_service<GetDomainName>(
    "guarded/get_domain_name",
    plansys2::guarded<GetDomainName>(
      node->get_logger(),
      [&](const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<GetDomainName::Request>,
      const std::shared_ptr<GetDomainName::Response> response) {
        response->name = "half done";  // Not kept when it throws
        if (calls++ % 2 == 0) {
          throw std::runtime_error("Empty predicate");
        }
        response->success = true;
      }));
  plansys2::testing::NodeSpinner spinner(node);

  for (int i = 0; i < 4; i++) {
    auto response = plansys2::testing::call<GetDomainName>(
      node, "guarded/get_domain_name", std::make_shared<GetDomainName::Request>());
    ASSERT_NE(response, nullptr) << i;
    if (i % 2 == 0) {
      EXPECT_FALSE(response->success);
      EXPECT_EQ(response->error_info, "Empty predicate");
      EXPECT_EQ(response->name, "");
    } else {
      EXPECT_TRUE(response->success);
      EXPECT_EQ(response->name, "half done");
    }
  }
}

// A response without success nor error_info: the default one
TEST(guarded_service, responses_without_success)
{
  auto node = rclcpp::Node::make_shared("guarded_plain_host");
  auto service = node->create_service<GetState>(
    "guarded/get_state",
    plansys2::guarded<GetState>(
      node->get_logger(),
      [](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<GetState::Request>,
      const std::shared_ptr<GetState::Response> response) {
        response->current_state.id = 3;
        throw std::out_of_range("vector");
      }));
  plansys2::testing::NodeSpinner spinner(node);
  auto response = plansys2::testing::call<GetState>(
    node, "guarded/get_state", std::make_shared<GetState::Request>());
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->current_state.id, 0);
}

// A deferred answer: the failed one is sent at once
TEST(guarded_service, deferred_answers)
{
  auto node = rclcpp::Node::make_shared("guarded_deferred_host");
  auto service = node->create_service<GetDomainName>(
    "guarded/deferred",
    plansys2::guarded_deferred<GetDomainName>(
      node->get_logger(),
      [](const std::shared_ptr<rclcpp::Service<GetDomainName>> service,
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<GetDomainName::Request>) {
        static int calls = 0;
        if (calls++ == 0) {
          throw std::runtime_error("Unknown object");
        }
        GetDomainName::Response response;
        response.success = true;
        service->send_response(*header, response);
      }));
  plansys2::testing::NodeSpinner spinner(node);
  for (bool expected : {false, true}) {
    auto response = plansys2::testing::call<GetDomainName>(
      node, "guarded/deferred", std::make_shared<GetDomainName::Request>());
    ASSERT_NE(response, nullptr);
    EXPECT_EQ(response->success, expected);
    EXPECT_EQ(response->error_info, expected ? "" : "Unknown object");
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
