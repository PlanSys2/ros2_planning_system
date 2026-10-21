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

// A domain change is done once the problem expert works with the new domain, whenever the
// problem expert was configured. Each scenario runs in its own process (see CMakeLists.txt).

#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_msgs/srv/change_domain.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;

class DomainChangeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pkgpath_ = plansys2::share_path("plansys2_problem_expert").string();
    simple_ = read("domain_simple.pddl");
    kitchen_ = read("domain_totally_different.pddl");

    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    domain_node_->set_parameter({"model_file", pkgpath_ + "/pddl/domain_simple.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath_ + "/pddl/domain_simple.pddl"});

    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);

    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    domain_client_ = std::make_shared<plansys2::DomainExpertClient>(client_node_);
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
  }

  void TearDown() override
  {
    spinner_.reset();
    domain_client_.reset();
    problem_client_.reset();
  }

  std::string read(const std::string & file)
  {
    std::ifstream ifs(pkgpath_ + "/pddl/" + file);
    return std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  }

  static void up(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  }

  // Whether the problem expert works with the kitchen domain
  bool in_kitchen()
  {
    bool ok = problem_client_->addInstance(plansys2::Instance("remy", "chef")) &&
      problem_client_->addInstance(plansys2::Instance("ratatouille", "dish")) &&
      problem_client_->addPredicate(plansys2::Predicate("(chef_cooking remy ratatouille)"));
    problem_client_->clearKnowledge();
    return ok;
  }

  bool in_simple()
  {
    bool ok = problem_client_->addInstance(plansys2::Instance("r2d2", "robot")) &&
      problem_client_->addInstance(plansys2::Instance("kitchen", "room")) &&
      problem_client_->addPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)"));
    problem_client_->clearKnowledge();
    return ok;
  }

  std::string pkgpath_;
  std::string simple_;
  std::string kitchen_;
  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

// Right after changeDomain() returns, the problem expert already takes the new domain:
// no need to wait, also over many changes
TEST_F(DomainChangeTest, new_domain_works_as_soon_as_the_change_returns)
{
  up(domain_node_);
  up(problem_node_);
  ASSERT_TRUE(in_simple());

  for (int i = 0; i < 60; i++) {
    ASSERT_TRUE(domain_client_->changeDomain(kitchen_)) << i;
    ASSERT_TRUE(in_kitchen()) << i;
    ASSERT_FALSE(in_simple()) << i;
    ASSERT_TRUE(domain_client_->changeDomain(simple_)) << i;
    ASSERT_TRUE(in_simple()) << i;
  }
}

// A problem expert configured after the change works with the new domain, also after being
// configured again, instead of the one of its model_file
TEST_F(DomainChangeTest, problem_expert_configured_late_takes_the_current_domain)
{
  up(domain_node_);
  ASSERT_TRUE(domain_client_->changeDomain(kitchen_));
  // Kept: the domain topic reaching the unconfigured problem expert is not observable
  std::this_thread::sleep_for(500ms);

  up(problem_node_);
  ASSERT_TRUE(in_kitchen());

  problem_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  problem_node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  up(problem_node_);
  ASSERT_TRUE(in_kitchen());
}

// The domain expert publishing again the domain the problem expert already has, as when it is
// activated again, changes nothing in the problem
TEST_F(DomainChangeTest, same_domain_again_keeps_the_problem)
{
  up(domain_node_);
  up(problem_node_);
  ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client_->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(problem_client_->addPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));

  domain_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  domain_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  // Kept: time for the replayed domain to arrive, which must change nothing
  std::this_thread::sleep_for(500ms);

  ASSERT_EQ(problem_client_->getInstances().size(), 2u);
  ASSERT_TRUE(problem_client_->existPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));
}

// A problem expert that cannot take the domain makes the change fail, and nothing changes
TEST(DomainChangeRejectedTest, domain_stays_if_the_problem_expert_rejects_it)
{
  std::string pkgpath = plansys2::share_path("plansys2_problem_expert").string();
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
  auto fake_problem_expert = rclcpp::Node::make_shared("fake_problem_expert");
  auto service = fake_problem_expert->create_service<plansys2_msgs::srv::ChangeDomain>(
    "problem_expert/update_domain",
    [](const std::shared_ptr<plansys2_msgs::srv::ChangeDomain::Request>,
    std::shared_ptr<plansys2_msgs::srv::ChangeDomain::Response> response) {
      response->success = false;
      response->error_info = "rejected";
    });

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(fake_problem_expert);
  plansys2::testing::ExecutorThread spinner(exe);

  domain_node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>(fake_problem_expert);
  ASSERT_EQ(domain_client->getName(), "simple");

  std::ifstream ifs(pkgpath + "/pddl/domain_totally_different.pddl");
  std::string kitchen((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(domain_client->changeDomain(kitchen));
  ASSERT_EQ(domain_client->getName(), "simple");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
