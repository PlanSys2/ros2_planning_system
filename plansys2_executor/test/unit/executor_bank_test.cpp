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

// Every solvable task of the bank that PlanSys2 loads is planned and executed to the end, and
// its goal holds in the problem expert afterwards

#include <filesystem>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "kairos/plan.hpp"
#include "kairos/validate.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_msgs/msg/execution_event.hpp"
#include "plansys2_pddl_parser/PddlBank.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;

namespace
{

// Not executable yet: none
const std::set<std::string> kNotLoaded = {};

// Does any action at once: the executor applies its effects
class Performer : public plansys2::ActionExecutorClient
{
public:
  Performer(const std::string & node_name, const std::string & action)
  : ActionExecutorClient(node_name)
  {
    set_parameter({"action_name", action});
    set_parameter({"rate", 50.0});
  }

private:
  void do_work() override {finish(true, 1.0, "done");}
};

// It lasts what the plan gives it, in the time of the test
class TimedPerformer : public plansys2::ActionExecutorClient
{
public:
  TimedPerformer(const std::string & node_name, const std::string & action)
  : ActionExecutorClient(node_name)
  {
    set_parameter({"action_name", action});
    set_parameter({"rate", 50.0});
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    started_ = now();
    return ActionExecutorClient::on_activate(state);
  }

private:
  void do_work() override
  {
    if ((now() - started_).seconds() >= get_planned_duration()) {
      finish(true, 1.0, "done");
    }
  }

  rclcpp::Time started_;
};

// It takes a while, different for each performer and each time: what a real one does, not
// what the plan says
class VariedPerformer : public plansys2::ActionExecutorClient
{
public:
  VariedPerformer(const std::string & node_name, const std::string & action)
  : ActionExecutorClient(node_name), seed_(std::hash<std::string>{}(node_name))
  {
    set_parameter({"action_name", action});
    set_parameter({"rate", 50.0});
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    started_ = now();
    seed_ = seed_ * 6364136223846793005ULL + 1442695040888963407ULL;
    lasts_ = 0.1 + static_cast<double>((seed_ >> 33) % 1000) / 1000.0 * 1.5;
    return ActionExecutorClient::on_activate(state);
  }

private:
  void do_work() override
  {
    if ((now() - started_).seconds() >= lasts_) {
      finish(true, 1.0, "done");
    }
  }

  uint64_t seed_;
  double lasts_ {0.0};
  rclcpp::Time started_;
};

// How a task runs: with what, and what the world does meanwhile
enum class Performers {Instant, Timed, Varied};
struct Options
{
  std::string bt_builder = "STNBTBuilder";
  Performers performers = Performers::Instant;
  std::string planner = "KAIROS";
  std::function<void(plansys2::ProblemExpertClient &)> meanwhile;
  uint8_t expected = plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
};

// In the time of the test, ten times that of the wall: an action starts this late at most, and
// lasts this much more than planned (the executor ticks at 50 Hz, the performers at 50 Hz);
// twice that in a slow build (not optimized, or for coverage)
constexpr double kTiming = PLANSYS2_TEST_SLOWDOWN > 1 ? 2.0 : 1.0;
constexpr double kLate = 2.0 * kTiming;
constexpr double kLonger = 1.5 * kTiming;

// Time ten times faster than the wall: what a plan lasts (PDDL+, timed literals) does not slow
// the tests. Every node reads it with use_sim_time; each isolated test has its ROS domain
class FastClock
{
public:
  explicit FastClock(double factor = 10.0)
  : node_(rclcpp::Node::make_shared("bank_clock")),
    publisher_(node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10))
  {
    publish();
    thread_ = std::thread(
      [this, factor]() {
        while (running_) {
          std::this_thread::sleep_for(5ms);
          seconds_ += 0.005 * factor;
          publish();
        }
      });
  }

  ~FastClock() {stop();}

  // Before the nodes go: with Zenoh, a subscription destroyed while the process publishes on its
  // topic may deadlock in rmw_zenoh
  void stop()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  double seconds() const {return seconds_;}

private:
  void publish()
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(static_cast<int64_t>(seconds_ * 1e9));
    publisher_->publish(clock);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  std::atomic<double> seconds_ {1000.0};
  std::atomic<bool> running_ {true};
  std::thread thread_;
};

// The nodes of PlanSys2 for one task, with a performer per action
struct Run
{
  ~Run() {clock.stop();}

  FastClock clock;
  std::shared_ptr<plansys2::DomainExpertNode> domain_node;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node;
  std::shared_ptr<plansys2::PlannerNode> planner_node;
  std::shared_ptr<plansys2::ExecutorNode> executor_node;
  std::vector<std::shared_ptr<plansys2::ActionExecutorClient>> performers;
  rclcpp::executors::MultiThreadedExecutor exe;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner;
  // What the executor says it does
  rclcpp::Node::SharedPtr listener;
  rclcpp::Subscription<plansys2_msgs::msg::ExecutionEvent>::SharedPtr trace_sub;
  std::mutex trace_mutex;
  std::vector<plansys2_msgs::msg::ExecutionEvent> trace;
};

using Event = plansys2_msgs::msg::ExecutionEvent;

// The plan as its trace tells it: one start and one end for each action, in order, between the
// start and the end of the plan, each event numbered after the one before
void check_trace(
  const plansys2_msgs::msg::Plan & plan, const std::vector<Event> & trace, bool succeeded,
  const std::string & report)
{
  ASSERT_FALSE(trace.empty()) << report;
  EXPECT_EQ(trace.front().type, Event::PLAN_START) << report;
  EXPECT_EQ(trace.back().type, Event::PLAN_END) << report;
  EXPECT_EQ(trace.back().detail, succeeded ? "succeeded" : "failed") << report;
  std::map<std::string, std::pair<uint64_t, uint64_t>> happened;
  double shift = 0.0;
  for (size_t i = 0; i < trace.size(); i++) {
    if (i > 0) {
      EXPECT_EQ(trace[i].sequence, trace[i - 1].sequence + 1) << report;
    }
    // The plan only goes later, and a SHIFT says by how much
    EXPECT_GE(trace[i].shift, shift - 1e-9) << "the plan went earlier\n" << report;
    if (trace[i].type == Event::SHIFT) {
      EXPECT_GT(trace[i].shift, shift) << "a SHIFT that moves nothing\n" << report;
    }
    shift = trace[i].shift;
    if (trace[i].type == Event::START) {
      EXPECT_EQ(happened[trace[i].action].first, 0u) << trace[i].action << " starts twice";
      happened[trace[i].action].first = trace[i].sequence;
    } else if (trace[i].type == Event::END) {
      EXPECT_EQ(happened[trace[i].action].second, 0u) << trace[i].action << " ends twice";
      happened[trace[i].action].second = trace[i].sequence;
    } else if (trace[i].type == Event::WAIT) {
      EXPECT_FALSE(trace[i].detail.empty()) << "a wait says for what";
    }
  }
  if (succeeded) {
    EXPECT_EQ(happened.size(), plan.items.size()) << report;
  }
  for (const auto & [action, sequence] : happened) {
    if (succeeded || sequence.second) {
      EXPECT_GT(sequence.first, 0u) << action << " ended without starting\n" << report;
      EXPECT_GT(sequence.second, sequence.first) << action << "\n" << report;
    }
  }
}

// Executed at its times: each action starts when the plan says, not before, lasts what it says,
// and the execution, as a plan, solves the problem the planner had
void check_timing(
  const plansys2_msgs::msg::Plan & plan, const std::string & domain, const std::string & problem,
  const std::vector<Event> & trace, const std::string & report)
{
  struct Done
  {
    plansys2_msgs::msg::PlanItem item;
    size_t index;  // In the plan
    double start;
    double end;
  };
  std::vector<Done> done;
  std::map<std::string, size_t> by_id;
  for (const auto & event : trace) {
    if (event.type != Event::START && event.type != Event::END) {
      continue;
    }
    if (!by_id.count(event.action)) {
      const auto colon = event.action.rfind(':');
      const auto expression = event.action.substr(0, colon);
      const double time = std::stod(event.action.substr(colon + 1)) / 1000.0;
      for (size_t index = 0; index < plan.items.size(); index++) {
        const auto & item = plan.items[index];
        if (item.action == expression && std::abs(item.time - time) < 1e-3) {
          by_id[event.action] = done.size();
          done.push_back({item, index, 0.0, 0.0});
        }
      }
    }
    auto & d = done.at(by_id.at(event.action));
    (event.type == Event::START ? d.start : d.end) = rclcpp::Time(event.stamp).seconds();
  }
  ASSERT_EQ(done.size(), plan.items.size()) << report;

  // The plan starts when the action least late starts
  double origin = std::numeric_limits<double>::infinity();
  for (const auto & d : done) {
    origin = std::min(origin, d.start - d.item.time);
  }
  // Happenings of one tick share their time, the clock of the test goes in steps: among them,
  // the order of the trace, 2 ms apart
  for (const auto & d : done) {
    const double late = d.start - origin - d.item.time;
    EXPECT_LE(late, kLate) << d.item.action << " at " << d.item.time << "\n" << report;
    if (d.item.duration > 0.0) {
      EXPECT_GE(d.end - d.start, d.item.duration - 0.05) << d.item.action << "\n" << report;
      EXPECT_LE(d.end - d.start, d.item.duration + kLonger) << d.item.action << "\n" << report;
    }
  }
  std::vector<double> starts(done.size()), ends(done.size());
  double previous = -1.0;
  for (const auto & event : trace) {
    if (event.type != Event::START && event.type != Event::END) {
      continue;
    }
    const double executed = rclcpp::Time(event.stamp).seconds() - origin;
    const double time = executed <= previous + 1e-9 ? previous + 0.002 : executed;
    (event.type == Event::START ? starts : ends)[by_id.at(event.action)] = time;
    previous = time;
  }

  kairos::Plan as_executed;
  as_executed.temporal = true;
  for (size_t i = 0; i < done.size(); i++) {
    const auto & d = done[i];
    // What it lasted: an end waits for those the plan orders before it
    kairos::PlanStep step;
    step.time = starts[i];
    step.duration = d.item.duration > 0.0 ? ends[i] - starts[i] : 0.0;
    auto action = d.item.action.substr(1, d.item.action.size() - 2);
    std::transform(action.begin(), action.end(), action.begin(), ::tolower);
    std::istringstream words(action);
    words >> step.name;
    for (std::string arg; words >> arg; ) {
      step.args.push_back(arg);
    }
    as_executed.steps.push_back(step);
  }
  std::stable_sort(
    as_executed.steps.begin(), as_executed.steps.end(),
    [](const auto & a, const auto & b) {return a.time < b.time;});
  if (plan.has_end) {
    as_executed.end = std::max(plan.end, as_executed.makespan());
  }
  kairos::ValidateOptions options;
  options.duration_tolerance = kLonger + 0.1;
  const auto validation = kairos::validate_plan(domain, problem, as_executed, options);
  std::string why;
  for (const auto & diagnostic : validation.diagnostics) {
    why += diagnostic.message + "\n";
  }
  if (validation.status != kairos::ValidationStatus::Unsupported) {
    EXPECT_TRUE(validation.valid()) << "the execution as a plan: " << why << report;
  }
}

// A plan without times, run as soon as each action can: the order its actions ended in, when
// their effects apply, as a sequential plan, solves the problem. Their conditions hold from their
// start to their end: the executor checks them all along
void check_order(
  const plansys2_msgs::msg::Plan & plan, const std::string & domain, const std::string & problem,
  const std::vector<Event> & trace, const std::string & report)
{
  kairos::Plan as_executed;
  for (const auto & event : trace) {
    if (event.type != Event::END) {
      continue;
    }
    auto action = event.action.substr(1, event.action.rfind(')') - 1);
    std::transform(action.begin(), action.end(), action.begin(), ::tolower);
    kairos::PlanStep step;
    step.time = static_cast<double>(as_executed.steps.size());
    std::istringstream words(action);
    words >> step.name;
    for (std::string arg; words >> arg; ) {
      step.args.push_back(arg);
    }
    as_executed.steps.push_back(step);
  }
  ASSERT_EQ(as_executed.steps.size(), plan.items.size()) << report;
  const auto validation = kairos::validate_plan(domain, problem, as_executed);
  std::string why;
  for (const auto & diagnostic : validation.diagnostics) {
    why += diagnostic.message + "\n";
  }
  if (validation.status != kairos::ValidationStatus::Unsupported) {
    EXPECT_TRUE(validation.valid()) << "the order it ran in: " << why << report;
  }
}

// Runs the task to its end. Timed, its actions last what the plan gives them and run at their
// times; varied, they take what they take, as soon as they can
void execute(const parser::pddl::BankCase & task, const Options & options = {})
{
  const auto & bt_builder = options.bt_builder;
  const auto & meanwhile = options.meanwhile;
  const auto expected = options.expected;
  const bool timed = options.performers == Performers::Timed;
  Run run;
  run.domain_node = std::make_shared<plansys2::DomainExpertNode>();
  run.problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  run.planner_node = std::make_shared<plansys2::PlannerNode>();
  run.executor_node = std::make_shared<plansys2::ExecutorNode>();
  run.executor_node->set_parameter({"bt_builder_plugin", bt_builder});
  if (options.planner == "POPF") {
    run.planner_node->declare_parameter("POPF.plugin", "plansys2/POPFPlanSolver");
    run.planner_node->set_parameter({"plan_solver_plugins", std::vector<std::string>{"POPF"}});
  }
  run.domain_node->set_parameter({"model_file", task.domain_file.string()});
  run.problem_node->set_parameter({"model_file", task.domain_file.string()});
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> experts = {
    run.domain_node, run.problem_node, run.planner_node, run.executor_node};
  for (auto & node : experts) {
    node->set_parameter({"use_sim_time", true});
    run.exe.add_node(node->get_node_base_interface());
  }
  run.listener = rclcpp::Node::make_shared("bank_trace");
  run.trace_sub = run.listener->create_subscription<Event>(
    "executor/execution_trace", rclcpp::QoS(1000), [&run](Event::SharedPtr event) {
      std::lock_guard<std::mutex> lock(run.trace_mutex);
      run.trace.push_back(*event);
    });
  run.exe.add_node(run.listener);
  run.spinner = std::make_unique<plansys2::testing::ExecutorThread>(run.exe);
  ASSERT_TRUE(
    plansys2::testing::wait_for(
      [&]() {
        for (auto & node : experts) {
          if (node->now().seconds() < 999.0) {
            return false;
          }
        }
        return true;
      }, 5s));
  for (auto transition : {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE}) {
    for (auto & node : experts) {
      node->trigger_transition(transition);
    }
  }
  for (auto & node : experts) {
    ASSERT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      << node->get_name();
  }
  if (options.planner == "KAIROS") {
    run.planner_node->set_parameter({"KAIROS.anytime", false});  // The first plan is enough
  }

  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(client_node);
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(client_node);
  auto planner_client = std::make_shared<plansys2::PlannerClient>(client_node);
  auto executor_client = std::make_shared<plansys2::ExecutorClient>(client_node);

  auto actions = domain_client->getActions();
  for (const auto & action : domain_client->getDurativeActions()) {
    actions.push_back(action);
  }
  run.executor_node->set_parameter({"timed_execution", timed});
  // Timed, actions of the same kind overlap: one performer each, as robots have
  std::vector<std::pair<std::string, std::string>> performers;
  for (const auto & action : actions) {
    for (int i = 0; i < (options.performers == Performers::Instant ? 1 : 3); i++) {
      performers.push_back({"bank_" + action + (i ? "_" + std::to_string(i) : ""), action});
    }
  }
  for (const auto & [name, action] : performers) {
    std::shared_ptr<plansys2::ActionExecutorClient> performer;
    if (options.performers == Performers::Timed) {
      performer = std::make_shared<TimedPerformer>(name, action);
    } else if (options.performers == Performers::Varied) {
      performer = std::make_shared<VariedPerformer>(name, action);
    } else {
      performer = std::make_shared<Performer>(name, action);
    }
    performer->set_parameter({"use_sim_time", true});
    run.exe.add_node(performer->get_node_base_interface());
    performer->trigger_transition(Transition::TRANSITION_CONFIGURE);
    run.performers.push_back(performer);
  }

  ASSERT_TRUE(problem_client->addProblem(task.problem));
  const auto domain = domain_client->getDomain();
  const auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);
  ASSERT_TRUE(plan.has_value());

  // What went wrong, if anything: the plan and how each action ended
  std::string report = "plan:\n";
  for (const auto & item : plan->items) {
    report += "  " + std::to_string(item.time) + ": " + item.action + " [" +
      std::to_string(item.duration) + "]\n";
  }
  auto describe = [&report](const std::vector<plansys2_msgs::msg::ActionExecutionInfo> & s) {
      for (const auto & info : s) {
        report += "  " + info.action_full_name + " status " + std::to_string(info.status) +
          " " + info.message_status + " start " +
          std::to_string(rclcpp::Time(info.start_stamp).seconds()) + " end " +
          std::to_string(rclcpp::Time(info.status_stamp).seconds()) + " happened " +
          std::to_string(rclcpp::Time(info.start_happening_stamp).seconds()) + " to " +
          std::to_string(rclcpp::Time(info.end_happening_stamp).seconds()) + "\n";
      }
    };

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));
  if (meanwhile) {
    meanwhile(*problem_client);
  }
  const bool ended = plansys2::testing::wait_for(
    [&]() {return !executor_client->execute_and_check_plan();}, 20s);
  if (!ended) {
    describe(executor_client->getFeedBack().action_execution_status);
    report += "state:\n";
    for (const auto & predicate : problem_client->getPredicates()) {
      report += "  " + parser::pddl::toString(predicate) + "\n";
    }
    executor_client->cancel_plan_execution();
  }
  ASSERT_TRUE(ended) << report;
  auto result = executor_client->getResult();
  ASSERT_TRUE(result.has_value());
  describe(result->action_execution_status);
  EXPECT_EQ(result->result, expected) << report;
  if (expected == plansys2_msgs::action::ExecutePlan::Result::SUCCESS) {
    EXPECT_TRUE(problem_client->isGoalSatisfied(problem_client->getGoal())) << report;
  }
  // The trace of the plan: up to its end, which comes right after the result
  const bool succeeded = result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
  std::vector<Event> trace;
  plansys2::testing::wait_for(
    [&]() {
      std::lock_guard<std::mutex> lock(run.trace_mutex);
      trace = run.trace;
      return !trace.empty() && trace.back().type == Event::PLAN_END;
    }, 2s);
  check_trace(plan.value(), trace, succeeded, report);
  if (timed && succeeded) {
    check_timing(plan.value(), domain, problem, trace, report);
  }
  if (options.performers == Performers::Varied && succeeded) {
    check_order(plan.value(), domain, problem, trace, report);
  }

  run.spinner.reset();
}

}  // namespace

class ExecutorBankTest : public ::testing::TestWithParam<std::string>
{
};

class ExecutorBankAsap : public ::testing::TestWithParam<std::string>
{
};

TEST_P(ExecutorBankTest, every_loaded_task_runs_to_its_goal)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  ASSERT_GE(cases.size(), 60u);
  const auto & bt_builder = GetParam();

  int executed = 0;
  const auto start = std::chrono::steady_clock::now();
  for (const auto & task : cases) {
    if (!task.solvable || kNotLoaded.count(task.feature)) {
      continue;
    }
    // Actions that have to overlap are not for the simple builder
    if (bt_builder == "SimpleBTBuilder" && task.feature == "required-concurrency") {
      continue;
    }
    const char * only = std::getenv("PLANSYS2_BANK_CASE");  // To look at one
    if (only && task.name != only) {
      continue;
    }
    SCOPED_TRACE(task.name);
    Options options;
    options.bt_builder = bt_builder;
    execute(task, options);
    plansys2::drain_ros(50ms);
    executed++;
  }
  if (!std::getenv("PLANSYS2_BANK_CASE")) {
    EXPECT_GE(executed, 28);
  }
  const auto took = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
  EXPECT_LT(took.count(), 120.0);
}

// PDDL3: a hard constraint the world breaks while the plan runs fails the plan
TEST_P(ExecutorBankTest, a_violated_constraint_fails_the_plan)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  const auto task = std::find_if(
    cases.begin(), cases.end(), [](const auto & c) {return c.name == "trajectory-constraints/p2";});
  ASSERT_NE(task, cases.end());
  // (always (item_at book office)): somebody takes the book away
  Options options;
  options.bt_builder = GetParam();
  options.meanwhile = [](plansys2::ProblemExpertClient & problem) {
      ASSERT_TRUE(
        problem.removePredicate(parser::pddl::fromStringPredicate("(item_at book office)")));
    };
  options.expected = plansys2_msgs::action::ExecutePlan::Result::FAILURE;
  execute(*task, options);
}

// The tasks whose plans have times, run at them: what the performers take is what the plan says
TEST(ExecutorBankTimed, temporal_tasks_run_at_their_times)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  const std::set<std::string> temporal = {
    "durative-actions", "duration-inequalities", "required-concurrency", "continuous-effects",
    "processes-events", "timed-initial-literals", "timed-initial-fluents",
    "trajectory-constraints"};
  int executed = 0;
  for (const auto & task : cases) {
    if (!task.solvable || !temporal.count(task.feature)) {
      continue;
    }
    // The constraints of a sequential plan have no times
    if (task.feature == "trajectory-constraints" && task.domain.find(":durative-actions") ==
      std::string::npos)
    {
      continue;
    }
    const char * only = std::getenv("PLANSYS2_BANK_CASE");
    if (only && task.name != only) {
      continue;
    }
    SCOPED_TRACE(task.name);
    Options options;
    options.performers = Performers::Timed;
    execute(task, options);
    plansys2::drain_ros(50ms);
    executed++;
  }
  if (!std::getenv("PLANSYS2_BANK_CASE")) {
    EXPECT_GE(executed, 15);
  }
}

// The tasks without times, with performers that take what they take: as soon as each action can,
// in an order that solves the task, with both builders
TEST_P(ExecutorBankAsap, classical_tasks_run_in_a_valid_order)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  int executed = 0;
  for (const auto & task : cases) {
    // Durative actions have times: the timed pass is for them
    if (!task.solvable || task.domain.find(":durative-action") != std::string::npos ||
      task.domain.find(":process") != std::string::npos ||
      task.domain.find(":event") != std::string::npos)
    {
      continue;
    }
    const char * only = std::getenv("PLANSYS2_BANK_CASE");
    if (only && task.name != only) {
      continue;
    }
    SCOPED_TRACE(task.name);
    Options options;
    options.bt_builder = GetParam();
    options.performers = Performers::Varied;
    execute(task, options);
    plansys2::drain_ros(50ms);
    executed++;
  }
  if (!std::getenv("PLANSYS2_BANK_CASE")) {
    EXPECT_GE(executed, 30);
  }
}

INSTANTIATE_TEST_SUITE_P(
  asap, ExecutorBankAsap, ::testing::Values("STNBTBuilder", "SimpleBTBuilder"),
  [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

// What POPF plans, executed: its plans separate happenings by epsilon and order them its own
// way. What it does not plan is not for it
TEST(ExecutorBankPopf, popf_plans_run_to_their_goal)
{
  const auto cases = parser::pddl::read_pddl_bank(
    plansys2::share_path("plansys2_pddl_parser") /
    "bank");
  // As POPF says it supports them
  const std::set<std::string> supported = {
    "strips-typing", "equality", "universal-preconditions", "numeric-fluents",
    "durative-actions", "timed-initial-literals", "required-concurrency"};
  int executed = 0;
  for (const auto & task : cases) {
    if (!task.solvable || !supported.count(task.feature)) {
      continue;
    }
    const char * only = std::getenv("PLANSYS2_BANK_CASE");
    if (only && task.name != only) {
      continue;
    }
    SCOPED_TRACE(task.name);
    Options options;
    options.planner = "POPF";
    execute(task, options);
    plansys2::drain_ros(50ms);
    executed++;
  }
  if (!std::getenv("PLANSYS2_BANK_CASE")) {
    EXPECT_GE(executed, 12);
  }
}

INSTANTIATE_TEST_SUITE_P(
  builders, ExecutorBankTest, ::testing::Values("STNBTBuilder", "SimpleBTBuilder"),
  [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
