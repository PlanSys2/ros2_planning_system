// Copyright 2026 Juan Carlos Manzanares Serrano
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

// Problem expert: the updaters it loads (updater_plugins) and their lifecycle

#include <algorithm>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/PackagePaths.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemUpdater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;  // NOLINT
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

std::string pddl(const std::string & name)
{
  return plansys2::share_path("plansys2_problem_expert").string() + "/pddl/" + name;
}

std::string read(const std::string & name)
{
  std::ifstream ifs(pddl(name));
  return std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
}

// Records its calls; the class it is loaded as selects how it behaves
class FakeUpdater : public plansys2::ProblemUpdater
{
public:
  explicit FakeUpdater(std::string type, std::vector<std::string> & calls)
  : type_(std::move(type)), calls_(calls) {}

  bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & name,
    const plansys2::ProblemAccess & access) override
  {
    calls_.push_back(name + ".configure");
    if (type_ == "throws_configure") {
      throw std::runtime_error("broken");
    }
    node_ = node;
    access_ = access;
    name_ = name;
    return type_ != "fails_configure";
  }

  bool activate() override
  {
    calls_.push_back(name_ + ".activate");
    if (type_ == "writes") {
      plansys2_msgs::msg::ProblemChange r2d2, at;
      r2d2.type = r2d2.ADD_INSTANCE;
      r2d2.instance.name = "r2d2";
      r2d2.instance.type = "robot";
      at.type = at.ADD_INSTANCE;
      at.instance.name = "kitchen";
      at.instance.type = "room";
      plansys2_msgs::msg::ProblemChange robot_at;
      robot_at.type = robot_at.ADD_PREDICATE;
      robot_at.node = parser::pddl::fromStringPredicate("(robot_at r2d2 kitchen)");
      std::string error;
      const bool made = access_.problem->applyChanges({r2d2, at, robot_at}, error);
      access_.notify();
      return made;
    }
    return type_ != "fails_activate";
  }

  void deactivate() override {calls_.push_back(name_ + ".deactivate");}
  void cleanup() override {calls_.push_back(name_ + ".cleanup");}
  void domain_changed() override
  {
    calls_.push_back(name_ + ".domain_changed:" + access_.domain->getName());
  }

private:
  std::string type_, name_;
  std::vector<std::string> & calls_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  plansys2::ProblemAccess access_;
};

}  // namespace

class ProblemUpdaters : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  // updaters: name -> class
  std::shared_ptr<plansys2::ProblemExpertNode> expert(
    const std::vector<std::pair<std::string, std::string>> & updaters)
  {
    std::vector<std::string> names;
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("model_file", pddl("domain_simple.pddl")),
      rclcpp::Parameter("state_file", "")};
    for (const auto & [name, type] : updaters) {
      names.push_back(name);
      parameters.emplace_back(name + ".plugin", type);
    }
    parameters.emplace_back("updater_plugins", names);
    // <name>.plugin is declared by the problem expert, as it loads its updaters
    auto node = std::make_shared<plansys2::ProblemExpertNode>(
      rclcpp::NodeOptions().parameter_overrides(parameters));
    node->set_updater_factory(
      [this](const std::string & type) -> plansys2::ProblemUpdater::Ptr {
        if (type == "unknown") {
          throw std::runtime_error("no class " + type);  // As pluginlib
        }
        return std::make_shared<FakeUpdater>(type, calls_);
      });
    return node;
  }

  std::vector<std::string> calls_;
};

TEST_F(ProblemUpdaters, through_the_lifecycle_of_the_problem_expert)
{
  auto node = expert({{"a", "ok"}, {"b", "ok"}});
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_DEACTIVATE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_CLEANUP).id(),
    State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(calls_, (std::vector<std::string>{
    "a.configure", "b.configure", "a.activate", "b.activate", "a.deactivate", "b.deactivate",
    "a.cleanup", "b.cleanup"}));
}

TEST_F(ProblemUpdaters, one_that_cannot_be_configured_stops_the_problem_expert)
{
  for (const auto & type : {"fails_configure", "throws_configure", "unknown"}) {
    calls_.clear();
    auto node = expert({{"a", "ok"}, {"b", type}});
    EXPECT_EQ(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_UNCONFIGURED) << type;
    // The ones configured before it are unloaded
    EXPECT_EQ(calls_.front(), "a.configure") << type;
    EXPECT_EQ(calls_.back(), "a.cleanup") << type;
  }
}

TEST_F(ProblemUpdaters, one_that_cannot_be_activated_stops_the_activation)
{
  auto node = expert({{"a", "fails_activate"}});
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);
}

TEST_F(ProblemUpdaters, they_change_the_problem_directly)
{
  auto node = expert({{"a", "writes"}});
  auto listener = rclcpp::Node::make_shared("listener");
  int notifications = 0;
  auto sub = listener->create_subscription<std_msgs::msg::Empty>(
    "problem_expert/update_notify", 10, [&](std_msgs::msg::Empty::SharedPtr) {notifications++;});
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(listener);
  // The changes are applied at once: a single notification
  EXPECT_TRUE(plansys2::testing::wait_for(
      [&]() {executor.spin_some(); return notifications >= 1;}, 5s));
  executor.spin_some();
  EXPECT_EQ(notifications, 1);
}

TEST_F(ProblemUpdaters, they_hear_of_a_new_domain)
{
  auto node = expert({{"a", "ok"}});
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);
  // Stands in for the domain expert
  auto publisher_node = rclcpp::Node::make_shared("domain_publisher");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
    "domain_expert/domain", rclcpp::QoS(100).transient_local());
  std_msgs::msg::String domain;
  domain.data = read("domain_simple_v2_extra_requirement.pddl");
  publisher->publish(domain);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(publisher_node);
  const auto heard = [&]() {
      executor.spin_some();
      return std::find_if(
        calls_.begin(), calls_.end(), [](const std::string & call) {
          return call.rfind("a.domain_changed:", 0) == 0;
        }) != calls_.end();
    };
  EXPECT_TRUE(plansys2::testing::wait_for(heard, 5s));
}

TEST_F(ProblemUpdaters, none_by_default)
{
  auto node = std::make_shared<plansys2::ProblemExpertNode>();
  node->set_parameter({"model_file", pddl("domain_simple.pddl")});
  EXPECT_EQ(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(node->get_parameter("updater_plugins").as_string_array().empty());
}
