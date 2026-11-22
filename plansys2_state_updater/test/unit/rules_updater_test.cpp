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

// The RulesUpdater in-process: what the live tests cannot trigger at will (refused writes,
// domain changes, failed reloads)

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_msgs/msg/state_update.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "plansys2_state_updater/rules_updater.hpp"

using namespace std::chrono_literals;
using plansys2_msgs::msg::StateUpdate;

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

// Topics of its own: nothing else running here publishes on them
std::string prefix()
{
  return "/su_in_process_" + std::to_string(getpid());
}

std::string rules(bool with_leak = true)
{
  const std::string p = prefix();
  std::string yaml = "rules:\n";
  if (with_leak) {
    yaml +=
      "  - {name: leak, topic: " + p + "/leak, type: std_msgs/msg/Bool, field: data,\n"
      "     target: {predicate: '(leak_detected)'}}\n";
  }
  yaml +=
    "  - {name: battery, topic: " + p + "/battery, type: std_msgs/msg/Float32, field: data,\n"
    "     transform: {linear: {scale: 100.0}}, target: {function: '(energy)'},\n"
    "     policy: {mode: correct, min_change: 2.0}}\n"
    "  - {name: low, topic: " + p + "/battery, type: std_msgs/msg/Float32, field: data,\n"
    "     transform: {compare: {op: '<', value: 0.2}}, target: {predicate: '(low_battery)'}}\n"
    "  - {name: sonar, topic: " + p + "/sonar_ok, type: std_msgs/msg/Bool, field: data,\n"
    "     target: {predicate: '(sensor_ok sonar)'}}\n"
    "  - {name: contacts, topic: " + p + "/contacts, type: diagnostic_msgs/msg/DiagnosticArray,\n"
    "     field: 'status[*]',\n"
    "     target: {instance: {name: 'contact_{name}', type: '{hardware_id}'},\n"
    "              facts: ['(contact_at {self} wp1)']},\n"
    "     policy: {remove_when_missing: true, max_instances: 2}}\n";
  return yaml;
}

class RulesUpdaterTest : public ::testing::Test
{
protected:
  virtual std::string config() const {return rules();}
  virtual bool sim_time() const {return false;}

  void SetUp() override
  {
    config_file_ = (std::filesystem::temp_directory_path() /
      ("su_in_process_" + std::to_string(getpid()) + ".yaml")).string();
    write_rules(config());
    domain_text_ = read_file(std::string(CONFIG_DIR) + "/example_domain.pddl");
    domain_ = std::make_shared<plansys2::DomainExpert>(domain_text_);
    problem_ = std::make_shared<plansys2::ProblemExpert>(domain_);
    ASSERT_TRUE(problem_->addProblem(read_file(std::string(CONFIG_DIR) + "/example_problem.pddl")));

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"state_updater.config_file", config_file_},
        {"state_updater.timer_period", 0.05},
        {"state_updater.resync_period", 0.5},
        {"state_updater.retry_after", 0.3},
        {"use_sim_time", sim_time()}});
    // Its own clock, if simulated: no other simulation here gives it the time
    options.arguments({"--ros-args", "-r", "/clock:=" + prefix() + "/clock"});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "problem_expert", "su_in_process", options);
    // Declared by the problem expert node in a real launch
    node_->declare_parameter("measured_functions", std::vector<std::string>{});
    io_ = std::make_shared<rclcpp::Node>("io", "su_in_process");
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(io_);

    events_sub_ = io_->create_subscription<StateUpdate>(
      "/su_in_process/state_updater/events", 100,
      [this](StateUpdate::SharedPtr event) {events_.push_back(*event);});
    status_sub_ = io_->create_subscription<std_msgs::msg::String>(
      "/su_in_process/state_updater/status", 10,
      [this](std_msgs::msg::String::SharedPtr status) {status_ = status->data;});

    plansys2::ProblemAccess access;
    access.problem = problem_;
    access.domain = domain_;
    access.notify = [this]() {notifications_++;};
    ASSERT_TRUE(updater_.configure(node_, "state_updater", access));
    ASSERT_TRUE(updater_.activate());
  }

  void TearDown() override
  {
    updater_.deactivate();
    updater_.cleanup();
    std::filesystem::remove(config_file_);
  }

  void write_rules(const std::string & yaml)
  {
    std::ofstream(config_file_) << yaml;
  }

  bool spin_until(const std::function<bool()> & done, std::chrono::duration<double> timeout = 5s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      if (done()) {
        return true;
      }
    }
    return false;
  }

  // Republishes until done: the subscription may not be matched yet at first
  template<typename Message>
  bool publish_until(
    const std::string & topic, const Message & message, const std::function<bool()> & done,
    std::chrono::duration<double> timeout = 5s)
  {
    auto publisher = io_->create_publisher<Message>(prefix() + topic, 10);
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      publisher->publish(message);
      if (spin_until(done, 100ms)) {
        return true;
      }
    }
    return false;
  }

  bool holds(const std::string & predicate)
  {
    return problem_->existPredicate(parser::pddl::fromStringPredicate(predicate));
  }

  double energy()
  {
    return problem_->getFunction("(energy)")->value;
  }

  std::vector<StateUpdate> events_of(const std::string & rule) const
  {
    std::vector<StateUpdate> out;
    for (const auto & event : events_) {
      if (event.rule == rule) {
        out.push_back(event);
      }
    }
    return out;
  }

  std::string reload()
  {
    auto client = io_->create_client<std_srvs::srv::Trigger>(
      "/su_in_process/state_updater/reload");
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!spin_until([&future]() {return future.wait_for(0s) == std::future_status::ready;})) {
      return "no answer";
    }
    const auto response = future.get();
    return (response->success ? "ok: " : "failed: ") + response->message;
  }

  static diagnostic_msgs::msg::DiagnosticArray contacts(
    const std::vector<std::pair<std::string, std::string>> & seen)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    for (const auto & [name, type] : seen) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = name;
      status.hardware_id = type;
      array.status.push_back(status);
    }
    return array;
  }

  std::string config_file_;
  std::string domain_text_;
  std::shared_ptr<plansys2::DomainExpert> domain_;
  std::shared_ptr<plansys2::ProblemExpert> problem_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr io_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<StateUpdate>::SharedPtr events_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  plansys2_state_updater::RulesUpdater updater_;
  std::vector<StateUpdate> events_;
  std::string status_;
  int notifications_ {0};
};

}  // namespace

TEST_F(RulesUpdaterTest, writes_what_the_topics_say_and_what_there_was_before)
{
  std_msgs::msg::Bool leak;
  leak.data = true;
  ASSERT_TRUE(publish_until("/leak", leak, [this]() {return holds("(leak_detected)");}));
  EXPECT_GE(notifications_, 1);
  ASSERT_TRUE(spin_until([this]() {return !events_of("leak").empty();}));
  EXPECT_TRUE(events_of("leak")[0].written);
  EXPECT_FALSE(events_of("leak")[0].has_previous);  // Never written before

  // One cycle, one notification: energy and low_battery from the same message
  const int before = notifications_;
  std_msgs::msg::Float32 battery;
  battery.data = 0.1f;
  ASSERT_TRUE(publish_until("/battery", battery, [this]() {return holds("(low_battery)");}));
  EXPECT_NEAR(energy(), 10.0, 1e-3);
  EXPECT_EQ(notifications_, before + 1);
  ASSERT_TRUE(spin_until([this]() {return !events_of("battery").empty();}));
  const auto event = events_of("battery")[0];
  EXPECT_EQ(event.kind, StateUpdate::FUNCTION);
  EXPECT_TRUE(event.has_previous);
  EXPECT_DOUBLE_EQ(event.previous_value, 100.0);  // What the problem had: mode correct
  EXPECT_NEAR(event.value, 10.0, 1e-3);
}

TEST_F(RulesUpdaterTest, a_write_the_problem_refuses_is_tried_again)
{
  std_msgs::msg::Bool ok;
  ok.data = true;
  // sonar is not an object of the problem
  ASSERT_TRUE(publish_until("/sonar_ok", ok, [this]() {return !events_of("sonar").empty();}));
  EXPECT_FALSE(events_of("sonar")[0].written);
  EXPECT_FALSE(holds("(sensor_ok sonar)"));
  // Now it is: written in a while, without another message
  ASSERT_TRUE(problem_->addInstance(plansys2::Instance("sonar", "sensor")));
  ASSERT_TRUE(spin_until([this]() {return holds("(sensor_ok sonar)");}));
  EXPECT_TRUE(spin_until([this]() {return events_of("sonar").back().written;}));
}

TEST_F(RulesUpdaterTest, objects_of_the_types_the_messages_say)
{
  auto has = [this](const std::string & name) {return problem_->getInstance(name).has_value();};
  ASSERT_TRUE(publish_until("/contacts", contacts({{"1", "contact"}, {"2", "whale"}}),
    [&]() {return has("contact_1");}));
  EXPECT_TRUE(holds("(contact_at contact_1 wp1)"));
  EXPECT_FALSE(has("contact_2"));  // Not a type of the domain
  ASSERT_TRUE(spin_until([this]() {return status_.find("whale") != std::string::npos;}));

  // Not in the message: gone at once (remove_when_missing)
  ASSERT_TRUE(publish_until("/contacts", contacts({{"3", "contact"}}),
    [&]() {return has("contact_3") && !has("contact_1");}));
  EXPECT_FALSE(holds("(contact_at contact_1 wp1)"));

  ASSERT_TRUE(publish_until("/contacts",
    contacts({{"3", "contact"}, {"4", "contact"}, {"5", "contact"}}),
    [&]() {return has("contact_4");}));
  spin_until([]() {return false;}, 300ms);
  EXPECT_FALSE(has("contact_5"));
}

TEST_F(RulesUpdaterTest, a_reload_that_fails_keeps_the_rules)
{
  write_rules("rules:\n  - {name: broken, topic: /x, target: {nothing: here}}\n");
  const std::string failed = reload();
  EXPECT_EQ(failed.rfind("failed: the rules stay as they were", 0), 0u) << failed;
  std_msgs::msg::Bool leak;
  leak.data = true;
  ASSERT_TRUE(publish_until("/leak", leak, [this]() {return holds("(leak_detected)");}));

  write_rules(rules());
  EXPECT_EQ(reload(), "ok: 5 rules on 4 topics, 5 kept as they were");
  write_rules(rules(false));
  EXPECT_EQ(reload(), "ok: 4 rules on 3 topics, 4 kept as they were");
}

TEST_F(RulesUpdaterTest, a_rule_that_no_longer_fits_the_domain_waits_until_it_does)
{
  std::string without = domain_text_;
  const std::string low = "    (low_battery)\n";
  ASSERT_NE(without.find(low), std::string::npos);
  without.erase(without.find(low), low.size());
  ASSERT_TRUE(problem_->updateDomain(without));
  updater_.domain_changed();
  ASSERT_TRUE(spin_until([this]() {
      return status_.find("it does not fit the new domain") != std::string::npos;
    }));

  // The battery still writes; the low rule, nothing
  std_msgs::msg::Float32 battery;
  battery.data = 0.1f;
  ASSERT_TRUE(publish_until("/battery", battery, [this]() {return energy() < 50.0;}));
  EXPECT_TRUE(events_of("low").empty());

  ASSERT_TRUE(problem_->updateDomain(domain_text_));
  updater_.domain_changed();
  ASSERT_TRUE(publish_until("/battery", battery, [this]() {return holds("(low_battery)");}));
}

TEST_F(RulesUpdaterTest, dry_run_says_what_it_would_write)
{
  node_->set_parameter(rclcpp::Parameter("state_updater.dry_run", true));
  std_msgs::msg::Bool leak;
  leak.data = true;
  ASSERT_TRUE(publish_until("/leak", leak, [this]() {return !events_of("leak").empty();}));
  EXPECT_FALSE(events_of("leak")[0].written);
  EXPECT_FALSE(holds("(leak_detected)"));
  EXPECT_EQ(notifications_, 0);
}

class SimulatedTime : public RulesUpdaterTest
{
protected:
  std::string config() const override
  {
    return "rules:\n"
           "  - {name: leak, topic: " + prefix() + "/leak, type: std_msgs/msg/Bool, field: data,\n"
           "     target: {predicate: '(leak_detected)'},\n"
           "     policy: {stale_after: 2.0, on_stale: '(sensor_stale leak)'}}\n";
  }
  bool sim_time() const override {return true;}

  void tick(double seconds)
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
    clock_pub_->publish(clock);
    spin_until([]() {return false;}, 200ms);
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
};

TEST_F(SimulatedTime, the_rules_start_with_the_clock_and_follow_it_back)
{
  clock_pub_ = io_->create_publisher<rosgraph_msgs::msg::Clock>(prefix() + "/clock", 10);
  // No clock yet: nothing is taken, nothing is stale
  std_msgs::msg::Bool leak;
  leak.data = true;
  EXPECT_FALSE(publish_until("/leak", leak, [this]() {return holds("(leak_detected)");}, 1s));

  // The simulation is at 1000 s: it starts now, not 1000 s ago
  for (int i = 0; i < 5; i++) {
    tick(1000.0);
  }
  EXPECT_FALSE(holds("(sensor_stale leak)"));
  ASSERT_TRUE(publish_until("/leak", leak, [this]() {return holds("(leak_detected)");}));
  tick(1003.0);
  ASSERT_TRUE(spin_until([this]() {return holds("(sensor_stale leak)");}));

  // It starts again from 10 s: the status goes on, the leak speaks, and stale counts from there
  status_.clear();
  tick(10.0);
  ASSERT_TRUE(spin_until([this]() {return !status_.empty();}));
  ASSERT_TRUE(publish_until("/leak", leak, [this]() {return !holds("(sensor_stale leak)");}));
  tick(13.0);
  ASSERT_TRUE(spin_until([this]() {return holds("(sensor_stale leak)");}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
