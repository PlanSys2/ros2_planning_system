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

#include <unistd.h>

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <future>
#include <thread>
#include <chrono>

#include "ament_index_cpp/get_package_share_path.hpp"

#include "gtest/gtest.h"
#include "plansys2_popf_plan_solver/popf_plan_solver.hpp"

#include "pluginlib/class_loader.hpp"
#include "plansys2_core/PlanSolverBase.hpp"

void test_plan_generation(const std::string & argument = "")
{
  std::string pkgpath =
    ament_index_cpp::get_package_share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");
  node->set_parameter(rclcpp::Parameter("POPF.arguments", argument));

  auto plan = planner->getPlan(domain_str, problem_str, "generate_plan_good");

  ASSERT_TRUE(plan);
  ASSERT_EQ(plan.value().items.size(), 3);
  ASSERT_EQ(plan.value().items[0].action, "(move leia kitchen bedroom)");
  ASSERT_EQ(plan.value().items[1].action, "(approach leia bedroom jack)");
  ASSERT_EQ(plan.value().items[2].action, "(talk leia jack jack m1)");
}

std::optional<std::filesystem::path> test_folder_creation(
  const std::string & output_dir = "", const std::string & node_namespace = "")
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();

  planner->configure(node, "POPF");
  if (!output_dir.empty()) {
    node->set_parameter(rclcpp::Parameter("POPF.output_dir", output_dir));
  }

  return planner->create_folders(node_namespace);
}

TEST(popf_plan_solver, generate_plan_good)
{
  test_plan_generation();
}

TEST(popf_plan_solver, generate_plan_good_with_argument)
{
  test_plan_generation("-e");
}

TEST(popf_plan_solver, load_popf_plugin)
{
  try {
    pluginlib::ClassLoader<plansys2::PlanSolverBase> lp_loader(
      "plansys2_core", "plansys2::PlanSolverBase");
    plansys2::PlanSolverBase::Ptr plugin =
      lp_loader.createUniqueInstance("plansys2/POPFPlanSolver");
    ASSERT_TRUE(true);
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    ASSERT_TRUE(false);
  }
}

TEST(popf_plan_solver, check_1_ok_domain)
{
  std::string pkgpath =
    ament_index_cpp::get_package_share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_1_ok.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  bool result = planner->isDomainValid(domain_str, "check_1_ok_domain");

  ASSERT_TRUE(result);
}

TEST(popf_plan_solver, check_2_error_domain)
{
  std::string pkgpath =
    ament_index_cpp::get_package_share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_2_error.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  bool result = planner->isDomainValid(domain_str, "check_2_error_domain");

  ASSERT_FALSE(result);
}

TEST(popf_plan_solver, generate_plan_unsolvable)
{
  std::string pkgpath =
    ament_index_cpp::get_package_share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_2.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str);

  ASSERT_FALSE(plan);
}

TEST(popf_plan_solver, generate_plan_error)
{
  std::string pkgpath =
    ament_index_cpp::get_package_share_path("plansys2_popf_plan_solver").string();
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_3.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto plan = planner->getPlan(domain_str, problem_str);

  ASSERT_FALSE(plan);
}

TEST(popf_plan_solver, create_folder_default)
{
  const auto output_dir = test_folder_creation();
  ASSERT_TRUE(output_dir.has_value());
  EXPECT_EQ(output_dir.value(), std::filesystem::temp_directory_path());
}

TEST(popf_plan_solver, create_folder_custom_path)
{
  const auto test_path = std::filesystem::temp_directory_path() / "test" / "path" / "one";
  const auto output_dir = test_folder_creation(test_path);
  ASSERT_TRUE(output_dir.has_value());
  EXPECT_EQ(output_dir.value(), test_path);
}

TEST(popf_plan_solver, create_folder_custom_path_and_namespace)
{
  const auto test_path = std::filesystem::temp_directory_path() / "test" / "path" / "two";
  const auto test_namespace = "/test/node";
  const auto output_dir = test_folder_creation(test_path, test_namespace);
  ASSERT_TRUE(output_dir.has_value());
  EXPECT_EQ(output_dir.value(), test_path / "test" / "node");
}

TEST(popf_plan_solver, create_folder_filesystem_error)
{
  const auto test_path = std::filesystem::temp_directory_path() / "test" / "path" / "three";

  // Create a file at the test path to force a filesystem error
  std::ofstream out(test_path);
  out << "random text\n";
  out.close();

  const auto test_namespace = "/test/node";
  const auto output_dir = test_folder_creation(test_path, test_namespace);
  ASSERT_FALSE(output_dir.has_value());
}

std::string read_test_file(const std::string & name)
{
  std::string pkgpath =
    ament_index_cpp::get_package_share_path("plansys2_popf_plan_solver").string();
  std::ifstream ifs(pkgpath + "/pddl/" + name);
  return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
}

// Number of running processes whose command line mentions `pattern`
int count_processes_with(const std::string & pattern)
{
  int count = 0;
  for (const auto & entry : std::filesystem::directory_iterator("/proc")) {
    const auto name = entry.path().filename().string();
    if (name.find_first_not_of("0123456789") != std::string::npos ||
      std::stoi(name) == getpid())
    {
      continue;
    }
    std::ifstream cmdline_ifs(entry.path() / "cmdline");
    std::string cmdline((std::istreambuf_iterator<char>(cmdline_ifs)),
      std::istreambuf_iterator<char>());
    if (cmdline.find(pattern) != std::string::npos) {
      count++;
    }
  }
  return count;
}

// popf is started through `ros2 run`, so it is a grandchild of the solver: it must
// not survive the call (#418)
TEST(popf_plan_solver, timeout_is_enforced_and_popf_is_killed)
{
  const std::string ns = "popf_timeout_test_" + std::to_string(getpid());
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto start = std::chrono::steady_clock::now();
  auto plan = planner->getPlan(
    read_test_file("domain_simple.pddl"), read_test_file("problem_hard_unsolvable.pddl"),
    ns, rclcpp::Duration(2s));
  auto elapsed = std::chrono::steady_clock::now() - start;

  ASSERT_FALSE(plan);
  ASSERT_GE(elapsed, 2s);
  ASSERT_LT(elapsed, 5s);

  std::this_thread::sleep_for(500ms);
  ASSERT_EQ(count_processes_with(ns), 0);

  // The solver is still usable after a timeout
  auto good = planner->getPlan(
    read_test_file("domain_simple.pddl"), read_test_file("problem_simple_1.pddl"), ns);
  ASSERT_TRUE(good);
  ASSERT_EQ(good.value().items.size(), 3u);
}

TEST(popf_plan_solver, cancel_stops_popf)
{
  const std::string ns = "popf_cancel_test_" + std::to_string(getpid());
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::POPFPlanSolver>();
  planner->configure(node, "POPF");

  auto future = std::async(
    std::launch::async, [&]() {
      return planner->getPlan(
        read_test_file("domain_simple.pddl"), read_test_file("problem_hard_unsolvable.pddl"),
        ns, rclcpp::Duration(60s));
    });

  // Wait until popf is actually running
  auto start = std::chrono::steady_clock::now();
  while (count_processes_with(ns) == 0 && std::chrono::steady_clock::now() - start < 10s) {
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(count_processes_with(ns), 0);

  planner->cancel();
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  ASSERT_FALSE(future.get());

  std::this_thread::sleep_for(500ms);
  ASSERT_EQ(count_processes_with(ns), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
