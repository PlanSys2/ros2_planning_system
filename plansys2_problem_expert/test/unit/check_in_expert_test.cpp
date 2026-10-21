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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

using namespace std::chrono_literals;
using Transition = lifecycle_msgs::msg::Transition;

// check_in_expert gives, in one call, what check gives with one call per fact
class CheckInExpertTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string pkgpath =
      plansys2::share_path("plansys2_problem_expert").string();
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    domain_node_->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_derived.pddl"});
    problem_node_->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_derived.pddl"});
    domain_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    problem_node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    domain_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    problem_node_->trigger_transition(Transition::TRANSITION_ACTIVATE);

    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);

    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    ASSERT_TRUE(client_->addInstance(plansys2::Instance("r1", "robot")));
    ASSERT_TRUE(client_->addInstance(plansys2::Instance("p1", "person")));
    ASSERT_TRUE(client_->addInstance(plansys2::Instance("p2", "person")));
    // bathroom appears in no fact
    for (const auto & room : {"kitchen", "bedroom", "bathroom"}) {
      ASSERT_TRUE(client_->addInstance(plansys2::Instance(room, "room")));
    }
    ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(robot_at r1 kitchen)")));
    ASSERT_TRUE(client_->addPredicate(plansys2::Predicate("(person_at p1 bedroom)")));
    ASSERT_TRUE(client_->addFunction(plansys2::Function("(= (room_distance kitchen bedroom) 10)")));
  }

  void TearDown() override
  {
    client_.reset();
    spinner_.reset();
  }

  static plansys2_msgs::msg::Tree tree_of(const std::string & expression)
  {
    plansys2_msgs::msg::Tree ret;
    parser::pddl::fromString(ret, expression, false, plansys2_msgs::msg::Node::AND);
    return ret;
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> client_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

TEST_F(CheckInExpertTest, same_answer_as_fact_by_fact)
{
  const std::vector<std::pair<std::string, bool>> cases = {
    {"(robot_at r1 kitchen)", true},
    {"(robot_at r1 bedroom)", false},
    {"(not (robot_at r1 bedroom))", true},
    // Derived predicates
    {"(inferred-robot_at r1 kitchen)", true},
    {"(not (inferred-robot_at r1 bedroom))", true},
    {"(and (inferred-robot_at r1 kitchen) (inferred-person_at p1 bedroom))", true},
    // Quantifiers range over all the rooms, also bathroom, which has no facts
    {"(exists (?ro - room) (robot_at r1 ?ro))", true},
    {"(forall (?ro - room) (robot_at r1 ?ro))", false},
    {"(exists (?ro - room) (and (not (robot_at r1 ?ro)) (not (person_at p1 ?ro))))", true},
    {"(forall (?ro - room) (not (person_at p2 ?ro)))", true},
    {"(not (exists (?ro - room) (person_at p2 ?ro)))", true},
    {"(exists (?p - person) (inferred-person_at ?p bedroom))", true},
    // Numbers and imply
    {"(> (room_distance kitchen bedroom) 5)", true},
    {"(< (room_distance kitchen bedroom) 5)", false},
    {"(imply (robot_at r1 kitchen) (person_at p1 bedroom))", true},
    {"(imply (robot_at r1 kitchen) (person_at p1 kitchen))", false},
  };

  for (const auto & [expression, expected] : cases) {
    auto condition = tree_of(expression);
    EXPECT_EQ(plansys2::check(condition, client_), expected) << "fact by fact: " << expression;
    EXPECT_EQ(plansys2::check_in_expert(condition, client_), expected) << "in expert: " <<
      expression;
  }

  // Nothing to check holds
  EXPECT_TRUE(plansys2::check_in_expert(plansys2_msgs::msg::Tree(), client_));
}

TEST_F(CheckInExpertTest, one_call_for_many_objects)
{
  for (int i = 0; i < 60; i++) {
    ASSERT_TRUE(client_->addInstance(plansys2::Instance("room_" + std::to_string(i), "room")));
  }
  // Holds for every room: all of them are visited
  auto condition = tree_of("(forall (?ro - room) (not (person_at p2 ?ro)))");

  auto time = [&](auto && evaluate) {
      auto start = std::chrono::steady_clock::now();
      for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(evaluate());
      }
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count() / 5;
    };
  double fact_by_fact = time([&]() {return plansys2::check(condition, client_);});
  double in_expert = time([&]() {return plansys2::check_in_expert(condition, client_);});
  std::cout << "forall over 63 rooms: " << fact_by_fact << " ms fact by fact, " << in_expert <<
    " ms in the expert" << std::endl;
  EXPECT_LT(in_expert * 5, fact_by_fact);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
