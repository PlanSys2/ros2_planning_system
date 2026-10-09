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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <atomic>
#include <thread>

#include "ament_index_cpp/get_package_share_path.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include "plansys2_core/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST(domain_expert, lifecycle)
{
  {
    auto test_node = rclcpp::Node::make_shared("get_action_from_string");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::experimental::executors::EventsExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());

    bool finish = false;
    std::thread t([&]() {
        while (!finish) {exe.spin_some();}
      });

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_EQ(domain_client->getDomain(), domain_client->getDomain(true));
    auto domain_str = domain_client->getDomain();

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_processed.pddl");
    std::string domain_str_p((
        std::istreambuf_iterator<char>(domain_ifs_p)),
      std::istreambuf_iterator<char>());

    ASSERT_EQ(domain_str, domain_str_p);

    finish = true;
    t.join();
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, lifecycle_error)
{
  {
    auto test_node = rclcpp::Node::make_shared("get_action_from_string");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_2_error.pddl"});
    rclcpp::experimental::executors::EventsExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());

    bool finish = false;
    std::thread t([&]() {
        while (!finish) {exe.spin_some();}
      });

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    finish = true;
    t.join();
  }
  plansys2::drain_ros(200ms);
}


// change_domain: hot domain swap. See refactor_dynamic_domain.md for the design and
// for the threading fix these tests exercise (the deferred-response callback +
// background thread in change_domain_service_callback()).
//
// Unlike the tests above, these join their spinning thread via RAII rather than a
// manual `finish = true; t.join();` at the bottom: an ASSERT_* failure returns from
// the test immediately, and a std::thread destroyed while still joinable calls
// std::terminate(), aborting the whole binary instead of just failing this test.
class ExecutorSpinner
{
public:
  explicit ExecutorSpinner(rclcpp::Executor & exe)
  : finish_(false), thread_(
      [this, &exe]() {
        // spin_once blocks until there is work, so this does not burn a core
        while (!finish_) {exe.spin_once(std::chrono::milliseconds(10));}
      }) {}

  ~ExecutorSpinner()
  {
    finish_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::atomic<bool> finish_;
  std::thread thread_;
};

TEST(domain_expert, change_domain_valid)
{
  plansys2::drain_ros(300ms);
  {
    auto test_node = rclcpp::Node::make_shared("change_domain_valid");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::experimental::executors::EventsExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    ExecutorSpinner spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 1.0) {
        rate.sleep();
      }
    }

    ASSERT_EQ(domain_client->getName(), "plansys2");

    std::ifstream new_domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
    std::string new_domain((
        std::istreambuf_iterator<char>(new_domain_ifs)),
      std::istreambuf_iterator<char>());

    ASSERT_TRUE(domain_client->changeDomain(new_domain));

    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(domain_client->getName(), "charging");

    ASSERT_TRUE(domain_client->getPredicate("charger_at").has_value());
    ASSERT_FALSE(domain_client->getPredicate("robot_talk").has_value());
    const auto types = domain_client->getTypes();
    ASSERT_NE(std::find(types.begin(), types.end(), "waypoint"), types.end());
    ASSERT_EQ(std::find(types.begin(), types.end(), "person"), types.end());
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, change_domain_invalid_rejected)
{
  plansys2::drain_ros(300ms);
  {
    auto test_node = rclcpp::Node::make_shared("change_domain_invalid_rejected");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::experimental::executors::EventsExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    ExecutorSpinner spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 1.0) {
        rate.sleep();
      }
    }

    ASSERT_EQ(domain_client->getName(), "plansys2");

    std::ifstream new_domain_ifs(pkgpath + "/pddl/domain_2_error.pddl");
    std::string new_domain((
        std::istreambuf_iterator<char>(new_domain_ifs)),
      std::istreambuf_iterator<char>());

    ASSERT_FALSE(domain_client->changeDomain(new_domain));

    // Rejected before touching anything: node stays active, old domain is untouched.
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(domain_client->getName(), "plansys2");
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, change_domain_rejected_when_unconfigured)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    rclcpp::experimental::executors::EventsExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    ExecutorSpinner spinner(exe);

    // Node was never configured: domain_expert_ is still null.
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_FALSE(domain_client->changeDomain("(define (domain whatever))"));
  }
  plansys2::drain_ros(200ms);
}

std::string read_file(const std::string & path)
{
  std::ifstream ifs(path);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

TEST(domain_expert, change_domain_rejected_when_inactive)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    rclcpp::experimental::executors::EventsExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    ExecutorSpinner spinner(exe);

    const std::string new_domain = read_file(pkgpath + "/pddl/domain_charging.pddl");

    // Configured but never activated: domain_pub_ cannot publish the change
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_FALSE(domain_client->changeDomain(new_domain));
    ASSERT_EQ(domain_client->getName(), "plansys2");

    // Active: accepted
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_TRUE(domain_client->changeDomain(new_domain));
    ASSERT_EQ(domain_client->getName(), "charging");

    // Deactivated again: rejected, the last accepted domain stays
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    ASSERT_FALSE(
      domain_client->changeDomain(read_file(pkgpath + "/pddl/domain_simple.pddl")));
    ASSERT_EQ(domain_client->getName(), "charging");
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, change_domain_empty_rejected)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    rclcpp::experimental::executors::EventsExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    ExecutorSpinner spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    const auto types_before = domain_client->getTypes();
    ASSERT_FALSE(types_before.empty());

    for (const std::string empty : {"", " ", "\n", " \t\r\n "}) {
      ASSERT_FALSE(domain_client->changeDomain(empty));
      ASSERT_EQ(domain_client->getName(), "plansys2");
      ASSERT_EQ(domain_client->getTypes(), types_before);
    }
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, change_domain_sequence_publishes_each_accepted_domain)
{
  plansys2::drain_ros(300ms);
  {
    auto test_node = rclcpp::Node::make_shared("change_domain_sequence");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    std::string pkgpath =
      ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    std::mutex mutex;
    std::vector<std::string> published;
    auto sub = test_node->create_subscription<std_msgs::msg::String>(
      "domain_expert/domain", rclcpp::QoS(100).transient_local().reliable(),
      [&mutex, &published](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        published.push_back(msg->data);
      });

    rclcpp::experimental::executors::EventsExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(test_node);
    ExecutorSpinner spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    const std::string simple = read_file(pkgpath + "/pddl/domain_simple.pddl");
    const std::string charging = read_file(pkgpath + "/pddl/domain_charging.pddl");
    const std::string error = read_file(pkgpath + "/pddl/domain_2_error.pddl");

    auto wait_published = [&](size_t n) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < 5s) {
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (published.size() >= n) {
              return true;
            }
          }
          std::this_thread::sleep_for(10ms);
        }
        return false;
      };

    // Activation publishes the initial domain. Nodes left alive by earlier tests may
    // also replay their transient_local domains on match, so count from a baseline.
    ASSERT_TRUE(wait_published(1));
    std::this_thread::sleep_for(1s);
    size_t base;
    {
      std::lock_guard<std::mutex> lock(mutex);
      base = published.size();
    }

    ASSERT_TRUE(domain_client->changeDomain(charging));
    ASSERT_EQ(domain_client->getName(), "charging");
    ASSERT_TRUE(wait_published(base + 1));

    // A rejected change publishes nothing and keeps the current domain
    ASSERT_FALSE(domain_client->changeDomain(error));
    ASSERT_EQ(domain_client->getName(), "charging");

    ASSERT_TRUE(domain_client->changeDomain(simple));
    ASSERT_EQ(domain_client->getName(), "plansys2");
    ASSERT_TRUE(domain_client->getPredicate("robot_talk").has_value());
    ASSERT_TRUE(wait_published(base + 2));

    std::this_thread::sleep_for(300ms);
    const std::string served = domain_client->getDomain();
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(published.size(), base + 2);
    ASSERT_NE(published[base].find("charging"), std::string::npos);
    ASSERT_NE(published[base + 1].find("plansys2"), std::string::npos);
    // What is published is what get_domain serves
    ASSERT_EQ(published[base + 1], served);
  }
  plansys2::drain_ros(200ms);
}

// The POPF solver used for validation keeps a pointer to the node: it must not keep
// the node alive (#422)
TEST(domain_expert, domain_expert_node_is_destroyed)
{
  std::string pkgpath = ament_index_cpp::get_package_share_path("plansys2_domain_expert").string();

  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    std::weak_ptr<plansys2::DomainExpertNode> weak = domain_node;
    domain_node.reset();
    ASSERT_TRUE(weak.expired());
  }

  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    std::weak_ptr<plansys2::DomainExpertNode> weak = domain_node;
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(
      domain_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    domain_node.reset();
    ASSERT_TRUE(weak.expired());
  }

  // A domain that fails validation leaves the node unconfigured, but it must still go away
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    std::weak_ptr<plansys2::DomainExpertNode> weak = domain_node;
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_2_error.pddl"});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node.reset();
    ASSERT_TRUE(weak.expired());
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new ROS2Environment);

  return RUN_ALL_TESTS();
}
