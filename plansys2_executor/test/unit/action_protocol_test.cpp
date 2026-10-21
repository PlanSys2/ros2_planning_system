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

// The protocol between ActionExecutor and the performers, message by message.
// Each scenario runs in its own process (see CMakeLists.txt).

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT
using ActionExecution = plansys2_msgs::msg::ActionExecution;
using Transition = lifecycle_msgs::msg::Transition;
using Status = plansys2::ActionExecutor::Status;

namespace
{

const char kAction[] = "(move r2d2 wheels_zone assembly_zone)";

// A real performer that ends the action after a few cycles
class QuickMove : public plansys2::ActionExecutorClient
{
public:
  explicit QuickMove(const std::string & name)
  : ActionExecutorClient(name)
  {
    set_parameter({"action_name", "move"});
    set_parameter({"rate", 20.0});
  }

  std::atomic<int> executions {0};
  std::atomic<double> planned_duration {-1.0};

private:
  void do_work() override
  {
    planned_duration = get_planned_duration();
    if (++cycles_ >= 3) {
      cycles_ = 0;
      executions++;
      finish(true, 1.0, "done");
    } else {
      send_feedback(cycles_ / 3.0, "moving");
    }
  }

  int cycles_ {0};
};

// It lasts what the plan gives it, from its start
class TimedMove : public plansys2::ActionExecutorClient
{
public:
  explicit TimedMove(const std::string & name)
  : ActionExecutorClient(name)
  {
    set_parameter({"action_name", "move"});
    set_parameter({"rate", 20.0});
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    started_ = now();
    return ActionExecutorClient::on_activate(state);
  }

  std::atomic<double> planned_duration {-1.0};

private:
  void do_work() override
  {
    planned_duration = get_planned_duration();
    if ((now() - started_).seconds() >= get_planned_duration()) {
      finish(true, 1.0, "done");
    }
  }

  rclcpp::Time started_;
};

// A performer driven by hand: it answers requests after `response_delay` and does what the
// test says with the action it is confirmed for
class FakePerformer : public rclcpp::Node
{
public:
  explicit FakePerformer(const std::string & name, std::chrono::milliseconds response_delay = 0ms)
  : rclcpp::Node(name), response_delay_(response_delay)
  {
    pub_ = create_publisher<ActionExecution>("actions_hub", rclcpp::QoS(100).reliable());
    sub_ = create_subscription<ActionExecution>(
      "actions_hub", rclcpp::QoS(100).reliable(),
      [this](ActionExecution::SharedPtr msg) {on_message(*msg);});
  }

  void send(uint8_t type, bool success = false, float completion = 0.0f)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ActionExecution msg = confirmed_;
    msg.type = type;
    msg.node_id = get_name();
    msg.success = success;
    msg.completion = completion;
    pub_->publish(msg);
  }

  // Like send, but as if it belonged to another request
  void send_for_other_request(uint8_t type, bool success)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ActionExecution msg = confirmed_;
    msg.type = type;
    msg.node_id = get_name();
    msg.request_id = "another request";
    msg.success = success;
    pub_->publish(msg);
  }

  std::atomic<bool> answer {true};
  std::atomic<bool> fail_when_confirmed {false};
  std::atomic<int> requests {0};
  std::atomic<int> confirms {0};
  std::atomic<int> rejects {0};

private:
  void on_message(const ActionExecution & msg)
  {
    if (msg.type == ActionExecution::REQUEST && msg.action == "move") {
      requests++;
      if (!answer) {
        return;
      }
      auto response = msg;
      response.type = ActionExecution::RESPONSE;
      response.node_id = get_name();
      if (response_delay_ == 0ms) {
        pub_->publish(response);
      } else {
        delayed_ = create_wall_timer(
          response_delay_, [this, response]() {
            delayed_->cancel();
            pub_->publish(response);
          });
      }
    } else if (msg.type == ActionExecution::CONFIRM && msg.node_id == get_name()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        confirmed_ = msg;
      }
      confirms++;
      if (fail_when_confirmed) {
        send(ActionExecution::FINISH, false, 0.0f);
      }
    } else if (msg.type == ActionExecution::REJECT && msg.node_id == get_name()) {
      rejects++;
    }
  }

  std::chrono::milliseconds response_delay_;
  std::mutex mutex_;
  ActionExecution confirmed_;
  rclcpp::Publisher<ActionExecution>::SharedPtr pub_;
  rclcpp::Subscription<ActionExecution>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr delayed_;
};

}  // namespace

class ActionProtocolTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    requester_ = rclcpp_lifecycle::LifecycleNode::make_shared("executor");
    requester_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    requester_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    exe_.add_node(requester_->get_node_base_interface());

    recorder_ = rclcpp::Node::make_shared("hub_recorder");
    recorder_sub_ = recorder_->create_subscription<ActionExecution>(
      "actions_hub", rclcpp::QoS(100).reliable(),
      [this](ActionExecution::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(messages_mutex_);
        messages_.push_back(*msg);
      });
    exe_.add_node(recorder_);

    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  }

  // Nodes are destroyed once nothing spins them: destroying them while spinning may hang
  void TearDown() override
  {
    spinner_.reset();
    executors_.clear();
    nodes_.clear();
  }

  void add(rclcpp::Node::SharedPtr node)
  {
    nodes_.push_back(node);
    exe_.add_node(node);
  }

  std::shared_ptr<QuickMove> add_quick_move(const std::string & name)
  {
    auto node = std::make_shared<QuickMove>(name);
    nodes_.push_back(node);
    exe_.add_node(node->get_node_base_interface());
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    return node;
  }

  plansys2::ActionExecutor::Ptr new_executor()
  {
    executors_.push_back(plansys2::ActionExecutor::make_shared(kAction, requester_));
    return executors_.back();
  }

  // Ticks the executor, as the BT does, until the condition holds
  bool tick_until(
    const plansys2::ActionExecutor::Ptr & executor, const std::function<bool()> & condition,
    std::chrono::milliseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      executor->tick(requester_->now());
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  // Waits without ticking
  static bool wait_until(
    const std::function<bool()> & condition, std::chrono::milliseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  // Waits until the hub has the endpoints of `performers` performers and of the recorder
  bool hub_has(size_t performers)
  {
    return plansys2::testing::wait_for(
      [&]() {
        return recorder_->count_publishers("/actions_hub") >= performers &&
               recorder_->count_subscribers("/actions_hub") >= performers + 1;
      });
  }

  int count(uint8_t type)
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    int ret = 0;
    for (const auto & msg : messages_) {
      ret += msg.type == type;
    }
    return ret;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr requester_;
  rclcpp::Node::SharedPtr recorder_;
  rclcpp::Subscription<ActionExecution>::SharedPtr recorder_sub_;
  std::mutex messages_mutex_;
  std::vector<ActionExecution> messages_;
  std::vector<plansys2::ActionExecutor::Ptr> executors_;
  std::vector<std::shared_ptr<void>> nodes_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

// A performer that answers after the action is over is rejected, so it does not keep waiting
TEST_F(ActionProtocolTest, late_response_is_rejected)
{
  auto quick = add_quick_move("quick_move");
  auto late = std::make_shared<FakePerformer>("late_move", 1500ms);
  add(late);
  ASSERT_TRUE(hub_has(2));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s));
  ASSERT_EQ(late->rejects, 0);

  ASSERT_TRUE(wait_until([&]() {return late->rejects > 0;}, 3s));
  ASSERT_EQ(late->confirms, 0);
}

// A performer whose CONFIRM or REJECT is lost takes new requests after a while
TEST_F(ActionProtocolTest, performer_without_answer_is_available_again)
{
  auto quick = add_quick_move("quick_move");
  auto requester = rclcpp::Node::make_shared("silent_requester");
  auto pub = requester->create_publisher<ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable());
  add(requester);
  // The recorder and the performer get what it publishes
  ASSERT_TRUE(plansys2::testing::wait_for([&]() {return pub->get_subscription_count() >= 2u;}));

  ActionExecution request;
  request.type = ActionExecution::REQUEST;
  request.node_id = "silent_requester";
  request.request_id = "lost";
  request.action = "move";
  request.arguments = {"r2d2", "wheels_zone", "assembly_zone"};
  pub->publish(request);
  ASSERT_TRUE(wait_until([&]() {return count(ActionExecution::RESPONSE) == 1;}, 2s));

  // Nobody confirms nor rejects: it is committed, and then free again
  request.request_id = "next";
  pub->publish(request);
  std::this_thread::sleep_for(1s);
  ASSERT_EQ(count(ActionExecution::RESPONSE), 1);

  std::this_thread::sleep_for(5s);
  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s));
  ASSERT_EQ(quick->executions, 1);
}

// The same action again: the executor of the first time does not reject the performer of
// the second, which belongs to another request
TEST_F(ActionProtocolTest, same_action_twice)
{
  auto quick = add_quick_move("quick_move");
  ASSERT_TRUE(hub_has(1));

  for (int i = 1; i <= 3; i++) {
    auto executor = new_executor();
    ASSERT_TRUE(
      tick_until(
        executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s)) << i;
    ASSERT_EQ(quick->executions, i);
  }
  ASSERT_EQ(count(ActionExecution::REJECT), 0);
}

// The performer knows how long the plan gives the action, from the request on
TEST_F(ActionProtocolTest, the_performer_gets_the_planned_duration)
{
  auto quick = add_quick_move("quick_move");
  ASSERT_TRUE(hub_has(1));

  for (double duration : {4.25, 0.0, 86400.003}) {
    auto executor = new_executor();
    executor->set_planned_duration(duration);
    ASSERT_TRUE(
      tick_until(
        executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s));
    EXPECT_DOUBLE_EQ(quick->planned_duration, duration);
  }

  std::lock_guard<std::mutex> lock(messages_mutex_);
  int requests = 0;
  for (const auto & msg : messages_) {
    if (msg.type == ActionExecution::REQUEST || msg.type == ActionExecution::CONFIRM) {
      EXPECT_TRUE(
        msg.planned_duration == 4.25 || msg.planned_duration == 0.0 ||
        msg.planned_duration == 86400.003) << msg.planned_duration;
      requests++;
    }
  }
  EXPECT_GE(requests, 6);
}

// A plan replaced while it runs gives the running action another duration: its performer gets it
TEST_F(ActionProtocolTest, the_performer_gets_a_new_planned_duration)
{
  auto timed = std::make_shared<TimedMove>("timed_move");
  nodes_.push_back(timed);
  exe_.add_node(timed->get_node_base_interface());
  timed->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  executor->set_planned_duration(30.0);
  ASSERT_TRUE(
    tick_until(
      executor, [&]() {return timed->planned_duration == 30.0;}, 5s));
  ASSERT_EQ(executor->get_internal_status(), Status::RUNNING);

  // From 30 s to half a second: it ends soon
  executor->update_planned_duration(0.5);
  EXPECT_DOUBLE_EQ(executor->get_planned_duration(), 0.5);
  ASSERT_TRUE(
    tick_until(
      executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s));
  EXPECT_DOUBLE_EQ(timed->planned_duration, 0.5);

  std::lock_guard<std::mutex> lock(messages_mutex_);
  int updates = 0;
  for (const auto & msg : messages_) {
    if (msg.type == ActionExecution::UPDATE) {
      EXPECT_EQ(msg.node_id, "timed_move");
      EXPECT_DOUBLE_EQ(msg.planned_duration, 0.5);
      updates++;
    }
  }
  EXPECT_EQ(updates, 1);

  // Not running: nothing to tell
  auto idle = new_executor();
  idle->update_planned_duration(2.0);
  EXPECT_DOUBLE_EQ(idle->get_planned_duration(), 2.0);
}

// One performer for two actions: the second takes it as soon as the first is done, not at its
// next request, a second later
TEST_F(ActionProtocolTest, a_waiting_action_takes_the_performer_once_free)
{
  auto timed = std::make_shared<TimedMove>("timed_move");
  nodes_.push_back(timed);
  exe_.add_node(timed->get_node_base_interface());
  timed->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(hub_has(1));

  auto first = new_executor();
  first->set_planned_duration(1.5);
  ASSERT_TRUE(
    tick_until(first, [&]() {return first->get_internal_status() == Status::RUNNING;}, 5s));
  auto second = new_executor();
  second->set_planned_duration(0.1);
  second->tick(requester_->now());
  ASSERT_EQ(second->get_internal_status(), Status::DEALING);

  std::optional<std::chrono::steady_clock::time_point> first_done, second_running;
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline && !second_running) {
    first->tick(requester_->now());
    second->tick(requester_->now());
    if (!first_done && first->get_internal_status() == Status::SUCCESS) {
      first_done = std::chrono::steady_clock::now();
    }
    if (first_done && second->get_internal_status() == Status::RUNNING) {
      second_running = std::chrono::steady_clock::now();
    }
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(first_done && second_running);
  EXPECT_LT(*second_running - *first_done, 500ms);
}

// Two actions asked at once, two performers: both answer the first, the rejected one takes the
// second at once, not at the next retry
TEST_F(ActionProtocolTest, actions_asked_at_once_run_in_parallel)
{
  for (int i = 1; i <= 2; i++) {
    auto timed = std::make_shared<TimedMove>("timed_move_" + std::to_string(i));
    nodes_.push_back(timed);
    exe_.add_node(timed->get_node_base_interface());
    timed->trigger_transition(Transition::TRANSITION_CONFIGURE);
  }
  ASSERT_TRUE(hub_has(2));

  for (int round = 0; round < 3; round++) {
    auto first = new_executor();
    auto second = new_executor();
    first->set_planned_duration(1.5);
    second->set_planned_duration(1.5);

    const auto start = std::chrono::steady_clock::now();
    std::optional<std::chrono::steady_clock::time_point> both_running;
    while (std::chrono::steady_clock::now() < start + 3s && !both_running) {
      first->tick(requester_->now());
      second->tick(requester_->now());
      if (first->get_internal_status() == Status::RUNNING &&
        second->get_internal_status() == Status::RUNNING)
      {
        both_running = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(both_running) << "round " << round;
    EXPECT_LT(*both_running - start, 500ms) << "round " << round;

    ASSERT_TRUE(
      tick_until(first, [&]() {return first->get_internal_status() == Status::SUCCESS;}, 5s));
    ASSERT_TRUE(
      tick_until(second, [&]() {return second->get_internal_status() == Status::SUCCESS;}, 5s));
  }
}

// PDDL is case insensitive: a performer of Move runs move (#256)
TEST_F(ActionProtocolTest, action_names_are_case_insensitive)
{
  auto node = std::make_shared<QuickMove>("upper_case_move");
  node->set_parameter({"action_name", "Move"});
  nodes_.push_back(node);
  exe_.add_node(node->get_node_base_interface());
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(
      executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s));
  ASSERT_EQ(node->executions, 1);
}

// Two performers for the same action: one runs it, the other is rejected and stays free
TEST_F(ActionProtocolTest, second_performer_is_rejected_and_free)
{
  auto first = add_quick_move("move_1");
  auto second = add_quick_move("move_2");
  ASSERT_TRUE(hub_has(2));

  for (int i = 1; i <= 4; i++) {
    auto executor = new_executor();
    ASSERT_TRUE(
      tick_until(
        executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 5s)) << i;
  }
  ASSERT_EQ(first->executions + second->executions, 4);
  // Every request got a performer at once: nobody was left committed
  ASSERT_EQ(count(ActionExecution::REQUEST), 4);
}

// A FINISH after the action was cancelled does not change how it ended
TEST_F(ActionProtocolTest, finish_after_cancel_is_ignored)
{
  auto fake = std::make_shared<FakePerformer>("fake_move");
  add(fake);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::RUNNING;}, 3s));
  executor->cancel();
  fake->send(ActionExecution::FINISH, true, 1.0f);
  std::this_thread::sleep_for(500ms);
  ASSERT_EQ(executor->get_internal_status(), Status::CANCELLED);
}

// Messages for another request of the same action are not ours
TEST_F(ActionProtocolTest, messages_of_another_request_are_ignored)
{
  auto fake = std::make_shared<FakePerformer>("fake_move");
  add(fake);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::RUNNING;}, 3s));
  fake->send_for_other_request(ActionExecution::FINISH, true);
  std::this_thread::sleep_for(500ms);
  ASSERT_EQ(executor->get_internal_status(), Status::RUNNING);

  fake->send(ActionExecution::FINISH, true, 1.0f);
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 2s));
}

// A performer that fails before doing anything is replaced a few times, not for ever
TEST_F(ActionProtocolTest, activation_failures_are_bounded)
{
  auto fake = std::make_shared<FakePerformer>("failing_move");
  fake->fail_when_confirmed = true;
  add(fake);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::FAILURE;}, 10s));
  // The first time and three more
  ASSERT_EQ(fake->confirms, 4);
  std::this_thread::sleep_for(1500ms);
  ASSERT_EQ(fake->confirms, 4);
}

// An action may run long without feedback: it is not given to another performer meanwhile
TEST_F(ActionProtocolTest, long_action_without_feedback_runs_once)
{
  auto first = std::make_shared<FakePerformer>("silent_move");
  add(first);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::RUNNING;}, 3s));

  // Another performer appears, and no feedback comes for a long while
  auto second = std::make_shared<FakePerformer>("other_move");
  add(second);
  ASSERT_FALSE(tick_until(executor, [&]() {return second->confirms > 0;}, 8s));
  ASSERT_EQ(executor->get_internal_status(), Status::RUNNING);
  ASSERT_EQ(first->confirms, 1);

  first->send(ActionExecution::FINISH, true, 1.0f);
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::SUCCESS;}, 2s));
}

// The performer disappears in the middle of the action: it fails instead of waiting for ever
TEST_F(ActionProtocolTest, gone_performer_fails_the_action)
{
  // Not kept by the fixture: it is destroyed in the middle of the test
  auto fake = std::make_shared<FakePerformer>("doomed_move");
  exe_.add_node(fake);
  ASSERT_TRUE(hub_has(1));

  auto executor = new_executor();
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::RUNNING;}, 3s));
  fake->send(ActionExecution::FEEDBACK, false, 0.5f);
  std::this_thread::sleep_for(500ms);

  // Not destroyed while the executor may be serving it
  spinner_.reset();
  exe_.remove_node(fake);
  fake.reset();
  spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  ASSERT_TRUE(
    tick_until(executor, [&]() {return executor->get_internal_status() == Status::FAILURE;}, 8s));
  // The reason the controller is told: not its last feedback, but that it is gone
  EXPECT_EQ(executor->get_failure_reason(), "its performer doomed_move is gone");
  EXPECT_EQ(executor->get_feedback(), executor->get_failure_reason());
}

// Without performers, the action fails after 30 s even if the BT no longer ticks it, and it
// stops asking (#365)
TEST_F(ActionProtocolTest, no_performer_times_out_without_ticks)
{
  auto executor = new_executor();
  executor->tick(requester_->now());
  ASSERT_EQ(executor->get_internal_status(), Status::DEALING);

  ASSERT_TRUE(
    wait_until([&]() {return executor->get_internal_status() == Status::FAILURE;}, 33s));
  EXPECT_NE(executor->get_failure_reason().find("no performer took it"), std::string::npos);
  int requests = count(ActionExecution::REQUEST);
  std::this_thread::sleep_for(2500ms);
  ASSERT_EQ(count(ActionExecution::REQUEST), requests);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
