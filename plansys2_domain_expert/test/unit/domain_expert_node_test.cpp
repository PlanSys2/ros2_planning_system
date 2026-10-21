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
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <atomic>
#include <thread>

#include "plansys2_core/PackagePaths.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "plansys2_msgs/srv/validate_domain.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT


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
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ASSERT_EQ(domain_client->getDomain(), domain_client->getDomain(true));
    auto domain_str = domain_client->getDomain();

    std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_processed.pddl");
    std::string domain_str_p((
        std::istreambuf_iterator<char>(domain_ifs_p)),
      std::istreambuf_iterator<char>());

    ASSERT_EQ(domain_str, domain_str_p);
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, lifecycle_error)
{
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_2_error.pddl"});
    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    ASSERT_EQ(
    domain_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
  plansys2::drain_ros(200ms);
}

// A domain PlanSys2 cannot read fails at startup; those of the Kairos work it reads now
TEST(domain_expert, a_domain_plansys2_cannot_represent_fails_at_startup)
{
  const std::string pkgpath =
    plansys2::share_path("plansys2_domain_expert").string();
  {
    const auto file = (std::filesystem::temp_directory_path() / "domain_unknown_block.pddl");
    {
      std::ofstream out(file);
      out << "(define (domain d) (:requirements :strips) (:predicates (p))"
        " (:action a :parameters () :precondition (and) :effect (and (p))) (:axiom (p)))";
    }
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    domain_node->set_parameter({"model_file", file.string()});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }

  // Conditional effects, ?duration, PDDL+, constraints and preferences it represents now
  for (const char * file : {"domain_when.pddl", "domain_duration_effect.pddl",
      "domain_process.pddl", "domain_constraints.pddl", "domain_preference.pddl"})
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/" + std::string(file)});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) << file;
  }
  plansys2::drain_ros(200ms);
}

// change_domain: hot domain swap. See refactor_dynamic_domain.md for the design and
// for the threading fix these tests exercise (the deferred-response callback +
// background thread in change_domain_service_callback()).
TEST(domain_expert, change_domain_valid)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

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

TEST(domain_expert, get_constants_through_the_client)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_constants.pddl"});
    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    // The service did not exist: the client waited for it for ever
    ASSERT_EQ(domain_client->getConstants("robot"), std::vector<std::string>({"leia", "lema"}));
    ASSERT_EQ(domain_client->getConstants("person"), std::vector<std::string>({"jack", "john"}));
    const auto types = domain_client->getTypes();
    ASSERT_TRUE(domain_client->getConstants("spaceship").empty());
    ASSERT_TRUE(domain_client->getConstants("").empty());
    ASSERT_EQ(domain_client->getTypes(), types);
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, processes_and_events_through_the_client)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);
    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_process.pddl"});
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    EXPECT_EQ(domain_client->getProcesses(), std::vector<std::string>{"heat"});
    EXPECT_TRUE(domain_client->getEvents().empty());
    auto heat = domain_client->getProcess("heat", {"kitchen"});
    ASSERT_TRUE(heat);
    EXPECT_EQ(
      parser::pddl::toString(heat->effects),
      "(and (increase (temperature kitchen) (* #t 1.000000)))");
    EXPECT_FALSE(domain_client->getProcess("cool"));
    EXPECT_FALSE(domain_client->getEvent("heat"));
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, change_domain_invalid_rejected)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    rclcpp::executors::MultiThreadedExecutor exe;

    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

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
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    // Node was never configured: domain_expert_ is still null, the service answers no
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    auto request = std::make_shared<plansys2_msgs::srv::ChangeDomain::Request>();
    request->domain = "(define (domain whatever))";
    auto response = plansys2::testing::call<plansys2_msgs::srv::ChangeDomain>(
      client_node, "domain_expert/change_domain", request);
    ASSERT_NE(response, nullptr);
    ASSERT_FALSE(response->success);

    // A PlanSys2 client waits until it is active, and then goes on
    std::atomic<bool> done {false};
    std::string name;
    std::thread caller([&]() {
        name = domain_client->getName();
        done = true;
      });
    std::this_thread::sleep_for(2s);
    EXPECT_FALSE(done.load());
    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    EXPECT_TRUE(plansys2::testing::wait_for([&]() {return done.load();}, 10s));
    caller.join();
    EXPECT_EQ(name, "plansys2");
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
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    const std::string new_domain = read_file(pkgpath + "/pddl/domain_charging.pddl");

    // The service, as any node calls it: what it answers while not active
    auto change = [&](const std::string & domain) {
        auto request = std::make_shared<plansys2_msgs::srv::ChangeDomain::Request>();
        request->domain = domain;
        auto response = plansys2::testing::call<plansys2_msgs::srv::ChangeDomain>(
          client_node, "domain_expert/change_domain", request);
        return response && response->success;
      };

    // Configured but never activated: domain_pub_ cannot publish the change
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_FALSE(change(new_domain));

    // Active: accepted
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_TRUE(domain_client->changeDomain(new_domain));
    ASSERT_EQ(domain_client->getName(), "charging");

    // Deactivated again: rejected, the last accepted domain stays
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    ASSERT_FALSE(change(read_file(pkgpath + "/pddl/domain_simple.pddl")));
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(domain_client->getName(), "charging");
  }
  plansys2::drain_ros(200ms);
}

TEST(domain_expert, change_domain_empty_rejected)
{
  plansys2::drain_ros(300ms);
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto client_node = rclcpp::Node::make_shared("client_node");
    plansys2::testing::NodeSpinner client_node_spinner(client_node);
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

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
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);

    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

    std::mutex mutex;
    std::vector<std::string> published;
    auto sub = test_node->create_subscription<std_msgs::msg::String>(
      "domain_expert/domain", rclcpp::QoS(100).transient_local().reliable(),
      [&mutex, &published](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        published.push_back(msg->data);
      });

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(test_node);
    plansys2::testing::ExecutorThread spinner(exe);

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    const std::string simple = read_file(pkgpath + "/pddl/domain_simple.pddl");
    const std::string charging = read_file(pkgpath + "/pddl/domain_charging.pddl");
    const std::string error = read_file(pkgpath + "/pddl/domain_2_error.pddl");

    auto wait_published = [&](size_t n) {
        return plansys2::testing::wait_for(
          [&]() {
            std::lock_guard<std::mutex> lock(mutex);
            return published.size() >= n;
          }, 5s);
      };

    // Activation publishes the initial domain. Nodes left alive by earlier tests may
    // also replay their transient_local domains on match, so count from a baseline.
    ASSERT_TRUE(wait_published(1));
    // Kept: late replays have no end to wait for
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

    // Kept: time for an unexpected extra message to show up
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

// Nothing used for validation may keep the node alive (#422, when it was a POPF solver)
TEST(domain_expert, domain_expert_node_is_destroyed)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();

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

// Malformed domains used to call exit() inside the parser, killing the process (#416)
TEST(domain_expert, malformed_domains_do_not_kill_the_node)
{
  std::string pkgpath = plansys2::share_path("plansys2_domain_expert").string();

  // As model_file: configure fails, the process lives on
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_2_error.pddl"});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }

  // No predicates and an empty action is odd but valid PDDL (POPF rejected it, Kairos does not)
  {
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_no_predicates.pddl"});
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  // Through change_domain: rejected, the current domain stays
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  plansys2::testing::ExecutorThread spinner(exe);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  for (const std::string & domain : {
    read_file(pkgpath + "/pddl/domain_2_error.pddl"),
    std::string("hello world"),
    std::string("(define (domain d) (:requirements :strips)"),
    std::string("(")})
  {
    SCOPED_TRACE(domain.substr(0, 40));
    ASSERT_FALSE(domain_client->changeDomain(domain));
    ASSERT_EQ(domain_client->getName(), "plansys2");
  }
}

// Answers planner/validate_domain as the planner node would
class FakeValidator : public rclcpp::Node
{
public:
  FakeValidator()
  : Node("fake_domain_validator")
  {
    service_ = create_service<plansys2_msgs::srv::ValidateDomain>(
      "planner/validate_domain",
      [this](
        const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Request>,
        std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Response> response) {
        calls++;
        response->success = accept;
        response->error_info = accept ? "" : "rejected by the planner";
      });
  }

  std::atomic<bool> accept {true};
  std::atomic<int> calls {0};

private:
  rclcpp::Service<plansys2_msgs::srv::ValidateDomain>::SharedPtr service_;
};

// Validation waits for the planner inside a callback of the domain expert (the lifecycle and
// change_domain services): another thread of its executor brings the answer
TEST(domain_expert, validate_with_planner_from_a_callback)
{
  plansys2::drain_ros(300ms);
  {
    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto validator = std::make_shared<FakeValidator>();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    domain_node->set_parameter({"validate_using_planner_node", true});

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);
    // The planner runs apart, as in its own process
    rclcpp::executors::MultiThreadedExecutor planner_exe;
    planner_exe.add_node(validator);
    plansys2::testing::ExecutorThread planner_spinner(planner_exe);

    // Through the lifecycle service, so on_configure runs in a callback
    auto test_node = rclcpp::Node::make_shared("validate_from_a_callback_test");
    plansys2::testing::NodeSpinner test_node_spinner(test_node);
    auto change_state = test_node->create_client<lifecycle_msgs::srv::ChangeState>(
      "domain_expert/change_state");
    ASSERT_TRUE(change_state->wait_for_service(5s));
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    auto start = std::chrono::steady_clock::now();
    auto future = change_state->async_send_request(request);
    ASSERT_EQ(future.wait_for(10s), std::future_status::ready);
    ASSERT_TRUE(future.get()->success);
    ASSERT_LT(std::chrono::steady_clock::now() - start, 2s);
    ASSERT_EQ(validator->calls, 1);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    // A change of domain is validated the same way, inside a service callback
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>(test_node);
    std::ifstream new_domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
    std::string new_domain((
        std::istreambuf_iterator<char>(new_domain_ifs)),
      std::istreambuf_iterator<char>());
    start = std::chrono::steady_clock::now();
    ASSERT_TRUE(domain_client->changeDomain(new_domain));
    ASSERT_LT(std::chrono::steady_clock::now() - start, 2s);
    ASSERT_EQ(domain_client->getName(), "charging");
    ASSERT_EQ(validator->calls, 2);

    // The planner's verdict is kept
    validator->accept = false;
    std::ifstream old_domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
    std::string old_domain((
        std::istreambuf_iterator<char>(old_domain_ifs)),
      std::istreambuf_iterator<char>());
    ASSERT_FALSE(domain_client->changeDomain(old_domain));
    ASSERT_EQ(domain_client->getName(), "charging");
    ASSERT_EQ(validator->calls, 3);
  }
  plansys2::drain_ros(200ms);
}

// It used to wait for the planner forever
TEST(domain_expert, validate_with_planner_absent)
{
  plansys2::drain_ros(300ms);
  {
    std::string pkgpath =
      plansys2::share_path("plansys2_domain_expert").string();
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
    domain_node->set_parameter({"validate_using_planner_node", true});

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(domain_node->get_node_base_interface());
    plansys2::testing::ExecutorThread spinner(exe);

    auto start = std::chrono::steady_clock::now();
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_LT(std::chrono::steady_clock::now() - start, 8s);
    ASSERT_EQ(
      domain_node->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
  plansys2::drain_ros(200ms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new ROS2Environment);

  return RUN_ALL_TESTS();
}
