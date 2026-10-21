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

#include <filesystem>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_plan_array.hpp"
#include "plansys2_msgs/srv/validate_domain.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

namespace
{

std::string read_pddl(const std::string & name)
{
  const auto path =
    plansys2::share_path("plansys2_kairos_plan_solver") / "pddl" / name;
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

}  // namespace

// The planner node of PlanSys2 with Kairos as its solver, called through its services
class PlannerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_ = std::make_shared<plansys2::PlannerNode>();
    client_ = rclcpp::Node::make_shared("kairos_planner_client");
    get_plan_ = client_->create_client<plansys2_msgs::srv::GetPlan>("planner/get_plan");
    get_plan_array_ = client_->create_client<plansys2_msgs::srv::GetPlanArray>(
      "planner/get_plan_array");
    validate_ = client_->create_client<plansys2_msgs::srv::ValidateDomain>(
      "planner/validate_domain");
    exe_.add_node(planner_->get_node_base_interface());
    exe_.add_node(client_);
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  }

  void TearDown() override
  {
    spinner_.reset();
    planner_.reset();
  }

  void start(const std::vector<std::string> & ids, double timeout = 15.0)
  {
    planner_->set_parameter({"plan_solver_plugins", ids});
    planner_->set_parameter({"plan_solver_timeout", timeout});
    for (const auto & id : ids) {
      planner_->declare_parameter(
        id + ".plugin",
        std::string(id == "POPF" ? "plansys2/POPFPlanSolver" : "plansys2/KairosPlanSolver"));
    }
    planner_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    planner_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(planner_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(get_plan_->wait_for_service(5s));
  }

  template<typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> call(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    std::shared_ptr<typename ServiceT::Request> request)
  {
    auto future = client->async_send_request(request);
    if (future.wait_for(30s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> get_plan(
    const std::string & domain, const std::string & problem)
  {
    auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
    request->domain = domain;
    request->problem = problem;
    return call<plansys2_msgs::srv::GetPlan>(get_plan_, request);
  }

  std::shared_ptr<plansys2::PlannerNode> planner_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_;
  rclcpp::Client<plansys2_msgs::srv::GetPlanArray>::SharedPtr get_plan_array_;
  rclcpp::Client<plansys2_msgs::srv::ValidateDomain>::SharedPtr validate_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

// With no plan_solver_plugins the planner uses Kairos
TEST_F(PlannerNodeTest, default_solver_is_kairos)
{
  start({});
  auto response = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success) << response->error_info;
  ASSERT_EQ(response->plan.items.size(), 3u);

  auto unsolvable = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_2.pddl"));
  ASSERT_TRUE(unsolvable);
  EXPECT_FALSE(unsolvable->success);
  EXPECT_NE(unsolvable->error_info.find("KAIROS"), std::string::npos) << unsolvable->error_info;
}

TEST_F(PlannerNodeTest, plans_with_kairos)
{
  start({"KAIROS"});
  auto response = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success) << response->error_info;
  ASSERT_EQ(response->plan.items.size(), 3u);
  EXPECT_EQ(response->plan.items[0].action, "(move leia kitchen bedroom)");
  EXPECT_EQ(response->plan.items[2].action, "(talk leia jack jack m1)");

  // Replanning with the same domain, as the executor does
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 10; i++) {
    auto again = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_1.pddl"));
    ASSERT_TRUE(again && again->success);
  }
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), 2.0);
}

TEST_F(PlannerNodeTest, no_plan_reasons)
{
  start({"KAIROS"});
  auto unsolvable = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_2.pddl"));
  ASSERT_TRUE(unsolvable);
  EXPECT_FALSE(unsolvable->success);
  EXPECT_NE(unsolvable->error_info.find("KAIROS"), std::string::npos) << unsolvable->error_info;

  auto invalid = get_plan(read_pddl("domain_simple.pddl"), read_pddl("problem_simple_3.pddl"));
  ASSERT_TRUE(invalid);
  EXPECT_FALSE(invalid->success);
  EXPECT_FALSE(invalid->error_info.empty());
}

TEST_F(PlannerNodeTest, node_timeout)
{
  start({"KAIROS"}, 1.0);
  const auto start = std::chrono::steady_clock::now();
  auto response = get_plan(read_pddl("puzzle_domain.pddl"), read_pddl("puzzle_unsolvable.pddl"));
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  // Kairos returns before the node deadline: no cancel needed
  EXPECT_LT(elapsed, 1.5);
  EXPECT_NE(response->error_info.find("KAIROS"), std::string::npos) << response->error_info;
}

TEST_F(PlannerNodeTest, validate_domain)
{
  start({"KAIROS"});
  auto request = std::make_shared<plansys2_msgs::srv::ValidateDomain::Request>();
  request->domain = read_pddl("domain_1_ok.pddl");
  auto ok = call<plansys2_msgs::srv::ValidateDomain>(validate_, request);
  ASSERT_TRUE(ok);
  EXPECT_TRUE(ok->success);

  request->domain = read_pddl("domain_2_error.pddl");
  auto error = call<plansys2_msgs::srv::ValidateDomain>(validate_, request);
  ASSERT_TRUE(error);
  EXPECT_FALSE(error->success);
}

TEST_F(PlannerNodeTest, kairos_and_popf_together)
{
  start({"KAIROS", "POPF"});
  auto request = std::make_shared<plansys2_msgs::srv::GetPlanArray::Request>();
  request->domain = read_pddl("domain_simple.pddl");
  request->problem = read_pddl("problem_simple_1.pddl");
  auto response = call<plansys2_msgs::srv::GetPlanArray>(get_plan_array_, request);
  ASSERT_TRUE(response);
  ASSERT_TRUE(response->success) << response->error_info;
  ASSERT_EQ(response->plan_array.plan_array.size(), 2u);
  for (const auto & plan : response->plan_array.plan_array) {
    ASSERT_EQ(plan.items.size(), 3u);
    EXPECT_EQ(plan.items[1].action, "(approach leia bedroom jack)");
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
