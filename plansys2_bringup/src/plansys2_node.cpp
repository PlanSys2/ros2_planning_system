// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <sched.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <future>
#include <map>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_core/ClientNode.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_executor/ExecutorNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace
{

rclcpp::Logger logger() {return rclcpp::get_logger("plansys2");}

// --use-real-time [true|false] and --startup-timeout <seconds>, after the ROS arguments are out
struct Options
{
  bool real_time {false};
  int startup_timeout {5};
};

Options parse(const std::vector<std::string> & args)
{
  Options options;
  for (size_t i = 1; i < args.size(); i++) {
    if (args[i] == "--use-real-time") {
      options.real_time = true;
      if (i + 1 < args.size() && (args[i + 1] == "true" || args[i + 1] == "false")) {
        options.real_time = args[++i] == "true";
      }
    } else if (args[i] == "--startup-timeout" && i + 1 < args.size()) {
      options.startup_timeout = std::stoi(args[++i]);
    } else {
      RCLCPP_WARN(logger(), "Unknown argument ignored: %s", args[i].c_str());
    }
  }
  return options;
}

// The nodes are in this process: their transitions are triggered directly, no service, no node.
// A transition that does not end in time fails the startup instead of blocking it
bool transition(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::uint8_t transition,
  std::uint8_t expected, std::chrono::seconds timeout)
{
  auto done = std::make_shared<std::promise<std::uint8_t>>();
  auto result = done->get_future();
  std::thread(
    [node, transition, done]() {
      try {
        done->set_value(node->trigger_transition(transition).id());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Transition of %s failed: %s", node->get_name(), e.what());
        done->set_value(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN);
      }
    }).detach();
  if (result.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      logger(), "%s did not end its transition in %ld s", node->get_name(),
      static_cast<long>(timeout.count()));  // NOLINT
    return false;
  }
  if (result.get() != expected) {
    RCLCPP_ERROR(logger(), "%s did not reach the state %u", node->get_name(), expected);
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  const auto options = parse(rclcpp::remove_ros_arguments(argc, argv));

  if (options.real_time) {
    // Before any thread starts: they all inherit the policy
    sched_param sch {};
    sch.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      RCLCPP_ERROR(
        logger(), "Failed to set PlanSys2 to execute in Real Time: %s", std::strerror(errno));
      RCLCPP_ERROR(
        logger(), "Set use_real_time to false or give your system the permissions");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(logger(), "Set PlanSys2 to execute in Real Time");
  } else {
    RCLCPP_INFO(logger(), "Set PlanSys2 to execute with normal priority");
  }

  // The four nodes of PlanSys2, and no other
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  // The only place where PlanSys2 spins. One executor per node, as when launched apart: a plan
  // only blocks the planner. Multi-threaded: while a callback waits for a client of its node,
  // another thread brings the answer (the default group still takes one callback at a time)
  std::vector<std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>> executors;
  for (const auto & node : std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>{
    domain_node, problem_node, planner_node, executor_node})
  {
    executors.push_back(
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), plansys2::kComponentThreads));
    executors.back()->add_node(node->get_node_base_interface());
  }

  std::vector<std::thread> spin_threads;
  for (auto & exe : executors) {
    spin_threads.emplace_back(
      [exe]() {
        try {
          exe->spin();
        } catch (const std::exception & e) {
          // The executor's action server may throw while rclcpp shuts down: only then it is fine
          if (rclcpp::ok()) {
            RCLCPP_ERROR(logger(), "Spin failed: %s", e.what());
            rclcpp::shutdown();
          }
        }
      });
  }
  auto stop_spinning = [&]() {
      for (auto & exe : executors) {
        exe->cancel();
      }
      for (auto & thread : spin_threads) {
        thread.join();
      }
    };

  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;
  const std::chrono::seconds timeout(options.startup_timeout);
  // Configured in this order (the domain expert may validate with the planner), then activated
  bool started = true;
  for (const auto & node : std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>{
    planner_node, domain_node, problem_node, executor_node})
  {
    started = started && rclcpp::ok() &&
      transition(node, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE, timeout);
  }
  for (const auto & node : std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>{
    domain_node, problem_node, planner_node, executor_node})
  {
    started = started && rclcpp::ok() &&
      transition(node, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE, timeout);
  }
  if (!started) {
    RCLCPP_ERROR(logger(), "Failed to start plansys2!");
    stop_spinning();
    rclcpp::shutdown();
    return -1;
  }

  // Until shutdown: Ctrl-C ends every spin()
  for (auto & thread : spin_threads) {
    thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
