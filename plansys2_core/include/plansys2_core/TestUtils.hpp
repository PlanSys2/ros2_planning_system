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

#ifndef PLANSYS2_CORE__TESTUTILS_HPP_
#define PLANSYS2_CORE__TESTUTILS_HPP_

#include <atomic>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{
namespace testing
{

/**
 * @brief Spins an executor in a thread of its own while it exists.
 *
 * The thread stops and is joined when this object is destroyed, also when a failed ASSERT
 * returns early from a test: a joinable thread left behind aborts the whole test program.
 */
class ExecutorThread
{
public:
  explicit ExecutorThread(rclcpp::Executor & executor)
  : executor_(executor),
    thread_([this]() {
        while (!stop_ && rclcpp::ok()) {
          executor_.spin_some(std::chrono::milliseconds(10));
        }
      })
  {
  }

  ~ExecutorThread()
  {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ExecutorThread(const ExecutorThread &) = delete;
  ExecutorThread & operator=(const ExecutorThread &) = delete;

private:
  rclcpp::Executor & executor_;
  std::atomic<bool> stop_ {false};
  std::thread thread_;
};

/**
 * @brief Wait, without spinning, until a condition holds.
 *
 * @param[in] condition What to wait for; something else spins the nodes it depends on.
 * @param[in] timeout How long to wait at most.
 * @return Whether the condition holds.
 */
template<typename Condition>
bool wait_for(
  Condition condition, std::chrono::nanoseconds timeout = std::chrono::seconds(10))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() >= end || !rclcpp::ok()) {
      return condition();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

}  // namespace testing
}  // namespace plansys2

#endif  // PLANSYS2_CORE__TESTUTILS_HPP_
