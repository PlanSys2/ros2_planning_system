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

// Problem expert: goals after removing objects, derived predicates and subtypes, problems
// added as a whole, action expressions, and the node's configuration, cleanup and topics

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/ServiceCall.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace
{
std::string readFile(const std::string & name)
{
  const auto path =
    plansys2::share_path("plansys2_problem_expert").string() + "/pddl/" + name;
  std::ifstream ifs(path);
  return std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
}

std::shared_ptr<plansys2::ProblemExpert> expertWith(const std::string & domain_file)
{
  auto domain = std::make_shared<plansys2::DomainExpert>(readFile(domain_file));
  auto expert = std::make_shared<plansys2::ProblemExpert>(domain);
  for (const auto & [name, type] : std::vector<std::pair<std::string, std::string>>{
      {"r2d2", "robot"}, {"c3po", "robot"}, {"paco", "person"}, {"kitchen", "room"},
      {"bedroom", "room"}, {"hall", "room"}})
  {
    EXPECT_TRUE(expert->addInstance(plansys2::Instance(name, type)));
  }
  return expert;
}

std::string goalAfterRemoving(
  const std::string & goal, const std::string & object, const std::string & type = "room")
{
  auto expert = expertWith("domain_simple.pddl");
  EXPECT_TRUE(expert->setGoal(plansys2::Goal(goal))) << goal;
  EXPECT_TRUE(expert->removeInstance(plansys2::Instance(object, type)));
  return parser::pddl::toString(expert->getGoal());
}
}  // namespace

TEST(problem_expert_misc, goal_after_removing_an_object)
{
  // A conjunct that names it goes, the others stay
  EXPECT_EQ(
    goalAfterRemoving("(and (robot_at r2d2 kitchen)(robot_at c3po hall))", "hall"),
    "(and (robot_at r2d2 kitchen))");
  EXPECT_EQ(
    goalAfterRemoving("(and (robot_at r2d2 kitchen)(robot_at c3po hall))", "bedroom"),
    "(and (robot_at r2d2 kitchen)(robot_at c3po hall))");
  // All named it: no goal
  EXPECT_EQ(goalAfterRemoving("(and (robot_at r2d2 hall)(robot_at c3po hall))", "hall"), "");
  // A goal of one fact
  EXPECT_EQ(goalAfterRemoving("(robot_at r2d2 hall)", "hall"), "");
  EXPECT_EQ(goalAfterRemoving("(robot_at r2d2 kitchen)", "hall"), "(robot_at r2d2 kitchen)");
  // In an or, only the disjunct that names it
  EXPECT_EQ(
    goalAfterRemoving(
      "(and (or (robot_at r2d2 hall)(robot_at r2d2 kitchen))(robot_at c3po bedroom))", "hall"),
    "(and (or (robot_at r2d2 kitchen))(robot_at c3po bedroom))");
  EXPECT_EQ(
    goalAfterRemoving("(or (robot_at r2d2 hall)(robot_at r2d2 kitchen))", "hall"),
    "(or (robot_at r2d2 kitchen))");
  // A not, an exists or a comparison that names it goes as a whole
  EXPECT_EQ(
    goalAfterRemoving("(and (not (robot_at r2d2 hall))(robot_at c3po kitchen))", "hall"),
    "(and (robot_at c3po kitchen))");
  EXPECT_EQ(
    goalAfterRemoving(
      "(and (> (room_distance kitchen hall) 1)(robot_at c3po kitchen))", "hall"),
    "(and (robot_at c3po kitchen))");
  // A goal that does not name it stays, whatever its root
  EXPECT_EQ(
    goalAfterRemoving("(exists (?r - room) (robot_at r2d2 ?r))", "hall"),
    "(exists (?r - room) (robot_at r2d2 ?r))");
  EXPECT_EQ(
    goalAfterRemoving("(not (robot_at r2d2 kitchen))", "hall"), "(not (robot_at r2d2 kitchen))");
  EXPECT_EQ(
    goalAfterRemoving(
      "(and (exists (?r - room) (and (robot_at r2d2 ?r)(robot_at c3po hall)))"
      "(robot_at c3po kitchen))", "hall"),
    "(and (robot_at c3po kitchen))");
  // Removing a robot
  EXPECT_EQ(
    goalAfterRemoving("(and (robot_at r2d2 kitchen)(robot_at c3po hall))", "c3po", "robot"),
    "(and (robot_at r2d2 kitchen))");
}

TEST(problem_expert_misc, derived_predicates_of_subtypes)
{
  auto expert = expertWith("domain_simple_derived.pddl");
  ASSERT_TRUE(expert->addInstance(plansys2::Instance("lab", "room_with_teleporter")));
  ASSERT_TRUE(expert->addPredicate(plansys2::Predicate("(robot_at r2d2 lab)")));
  ASSERT_TRUE(expert->addPredicate(plansys2::Predicate("(robot_at c3po kitchen)")));

  std::vector<std::string> inferred;
  for (const auto & p : expert->getPredicates()) {
    if (p.name == "inferred-robot_at") {inferred.push_back(parser::pddl::toString(p));}
  }
  std::sort(inferred.begin(), inferred.end());
  EXPECT_EQ(
    inferred,
    std::vector<std::string>({"(inferred-robot_at c3po kitchen)", "(inferred-robot_at r2d2 lab)"}));
}

TEST(problem_expert_misc, a_problem_is_added_as_a_whole)
{
  auto expert = expertWith("domain_simple.pddl");
  ASSERT_TRUE(expert->addPredicate(plansys2::Predicate("(robot_at r2d2 kitchen)")));
  ASSERT_TRUE(expert->setGoal(plansys2::Goal("(and (robot_at r2d2 bedroom))")));
  const auto instances = expert->getInstances().size();
  const auto predicates = expert->getPredicates().size();

  const std::string head =
    "(define (problem p) (:domain simple)"
    "(:objects leia - robot lab kitchen - room)"
    "(:init (robot_at leia lab) (= (room_distance lab kitchen) 3))"
    "(:goal (and (robot_at leia kitchen)))";
  // The metric fails after objects, facts and goal were converted: none of them is kept
  EXPECT_FALSE(expert->addProblem(head + "(:metric minimize (room_distance leia lab)))"));
  EXPECT_EQ(expert->getInstances().size(), instances);
  EXPECT_FALSE(expert->existInstance("leia"));
  EXPECT_EQ(expert->getPredicates().size(), predicates);
  EXPECT_TRUE(expert->getFunctions().empty());
  EXPECT_EQ(parser::pddl::toString(expert->getGoal()), "(and (robot_at r2d2 bedroom))");

  // The same problem with a right metric is added
  EXPECT_TRUE(expert->addProblem(head + "(:metric minimize (room_distance lab kitchen)))"));
  EXPECT_TRUE(expert->existInstance("leia"));
  EXPECT_EQ(parser::pddl::toString(expert->getGoal()), "(and (robot_at leia kitchen))");

  // And again: what is there is not an error
  EXPECT_TRUE(expert->addProblem(head + ")"));
}

TEST(problem_expert_misc, names_are_case_insensitive)
{
  // A predicate written in another case is the domain's (#256)
  auto domain = std::make_shared<plansys2::DomainExpert>(
    "(define (domain d) (:requirements :strips :typing) (:types robot)"
    "(:predicates (notFilling ?r - robot))"
    "(:action fill :parameters (?r - robot) :precondition (notFilling ?r)"
    " :effect (not (notFilling ?r))))");
  plansys2::ProblemExpert expert(domain);
  ASSERT_TRUE(expert.addInstance(plansys2::Instance("R2D2", "robot")));
  ASSERT_TRUE(expert.addPredicate(plansys2::Predicate("(notFilling R2D2)")));
  EXPECT_TRUE(expert.existPredicate(plansys2::Predicate("(notfilling r2d2)")));
  EXPECT_TRUE(plansys2::check(parser::pddl::fromString("(notfilling r2d2)"), expert));
  auto action = domain->getAction("fill", {"r2d2"});
  ASSERT_NE(action, nullptr);
  EXPECT_TRUE(plansys2::check(action->preconditions, expert));
  EXPECT_TRUE(expert.removePredicate(plansys2::Predicate("(NOTFILLING r2d2)")));
  EXPECT_FALSE(expert.existPredicate(plansys2::Predicate("(notFilling R2D2)")));
}

TEST(problem_expert_misc, action_expressions)
{
  EXPECT_EQ(plansys2::get_action_name("(move r2d2 kitchen bedroom)"), "move");
  EXPECT_EQ(
    plansys2::get_action_params("(move r2d2 kitchen bedroom)"),
    std::vector<std::string>({"r2d2", "kitchen", "bedroom"}));
  EXPECT_TRUE(plansys2::get_action_params("(move)").empty());
  EXPECT_EQ(plansys2::get_action_name("(move)"), "move");
  EXPECT_EQ(
    plansys2::get_action_params("(move  r2d2   kitchen )"),
    std::vector<std::string>({"r2d2", "kitchen"}));
  EXPECT_EQ(
    plansys2::get_action_params("(move\tr2d2\nkitchen)"),
    std::vector<std::string>({"r2d2", "kitchen"}));
  EXPECT_EQ(
    plansys2::get_action_params("(move r2d2 kitchen):5"),
    std::vector<std::string>({"r2d2", "kitchen"}));
  EXPECT_EQ(plansys2::get_action_time("(move r2d2 kitchen):5"), 5);
  EXPECT_EQ(plansys2::get_action_time("(move r2d2 kitchen)"), -1);
}

class ProblemExpertNodeMisc : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static uint8_t configure(const std::string & model_file)
  {
    auto node = std::make_shared<plansys2::ProblemExpertNode>();
    node->set_parameter({"model_file", model_file});
    return node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id();
  }

  static std::string pddl(const std::string & name)
  {
    return plansys2::share_path("plansys2_problem_expert").string() +
           "/pddl/" + name;
  }
};

TEST_F(ProblemExpertNodeMisc, model_file_must_be_readable)
{
  using lifecycle_msgs::msg::State;
  EXPECT_EQ(configure(""), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(configure(":"), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(configure("/nonexistent/domain.pddl"), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(
    configure(pddl("domain_simple.pddl") + ":/nonexistent/domain.pddl"),
    State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(configure(pddl("domain_simple.pddl")), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ProblemExpertNodeMisc, cleanup_and_function_topics)
{
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto test_node = rclcpp::Node::make_shared("problem_expert_misc_test");
  auto client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  domain_node->set_parameter({"model_file", pddl("domain_simple.pddl")});
  problem_node->set_parameter({"model_file", pddl("domain_simple.pddl")});
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  std::atomic<int> knowledge{0};
  std::atomic<int> problems{0};
  auto knowledge_sub = test_node->create_subscription<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge", rclcpp::QoS(100).transient_local(),
    [&](plansys2_msgs::msg::Knowledge::SharedPtr) {knowledge++;});
  auto problem_sub = test_node->create_subscription<plansys2_msgs::msg::Problem>(
    "problem_expert/problem", rclcpp::QoS(100),
    [&](plansys2_msgs::msg::Problem::SharedPtr) {problems++;});

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(test_node);
  plansys2::testing::ExecutorThread spinner(exe);

  auto wait_for = [](const std::atomic<int> & counter, int value) {
      return plansys2::testing::wait_for([&]() {return counter >= value;});
    };
  // problem_expert/problem is volatile: messages before matching are lost
  ASSERT_TRUE(
    plansys2::testing::wait_for(
      [&]() {return test_node->count_publishers("problem_expert/problem") > 0;}));

  ASSERT_TRUE(client->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(wait_for(knowledge, 2));
  const int base_knowledge = knowledge;
  const int base_problems = problems;

  ASSERT_TRUE(client->addFunction(plansys2::Function("(= (room_distance kitchen bedroom) 3)")));
  ASSERT_TRUE(wait_for(knowledge, base_knowledge + 1));
  // Updating and removing a function are told as adding one
  ASSERT_TRUE(
    client->updateFunction(plansys2::Function("(= (room_distance kitchen bedroom) 5)")));
  EXPECT_TRUE(wait_for(knowledge, base_knowledge + 2));
  EXPECT_TRUE(wait_for(problems, base_problems + 2));
  ASSERT_TRUE(
    client->removeFunction(plansys2::Function("(= (room_distance kitchen bedroom) 5)")));
  EXPECT_TRUE(wait_for(knowledge, base_knowledge + 3));
  EXPECT_TRUE(wait_for(problems, base_problems + 3));

  // Cleaned up, the knowledge goes and the services say they are not active (kNotActive, which
  // a PlanSys2 client waits on: asked here as any node asks)
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  auto add = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  add->param.name = "hall";
  add->param.type = "room";
  auto added = plansys2::testing::call<plansys2_msgs::srv::AffectParam>(
    test_node, "problem_expert/add_problem_instance", add);
  ASSERT_NE(added, nullptr);
  EXPECT_FALSE(added->success);
  EXPECT_EQ(added->error_info, plansys2::kNotActive);
  auto exist = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  exist->node.node_type = plansys2_msgs::msg::Node::PREDICATE;
  exist->node.name = "robot_at";
  auto existed = plansys2::testing::call<plansys2_msgs::srv::ExistNode>(
    test_node, "problem_expert/exist_problem_predicate", exist);
  ASSERT_NE(existed, nullptr);
  EXPECT_FALSE(existed->exist);
  EXPECT_EQ(existed->error_info, plansys2::kNotActive);
  // Configured again, it starts from nothing
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_FALSE(client->getInstance("kitchen").has_value());
  EXPECT_TRUE(client->addInstance(plansys2::Instance("hall", "room")));
}

// Requests from hostile input (found fuzzing the services): the parser throws, and the
// exception took down the process; now each is a failed answer and the node goes on
TEST_F(ProblemExpertNodeMisc, hostile_requests_are_failed_answers)
{
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto test_node = rclcpp::Node::make_shared("problem_expert_hostile_test");
  auto client = std::make_shared<plansys2::ProblemExpertClient>(test_node);
  domain_node->set_parameter({"model_file", pddl("domain_simple.pddl")});
  problem_node->set_parameter({"model_file", pddl("domain_simple.pddl")});
  for (auto node : std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr>{
    domain_node, problem_node})
  {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(test_node);
  plansys2::testing::ExecutorThread spinner(exe);

  for (const char * expression : {"", "   ", "()", "(", ")", "(())", ","}) {
    SCOPED_TRACE(std::string("[") + expression + "]");
    for (const char * service : {
      "problem_expert/get_problem_predicate", "problem_expert/get_problem_function"})
    {
      auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();
      request->expression = expression;
      auto response = plansys2::testing::call<plansys2_msgs::srv::GetNodeDetails>(
        test_node, service, request);
      ASSERT_NE(response, nullptr) << service;
      EXPECT_FALSE(response->success) << service;
    }
  }
  // Still serving
  EXPECT_TRUE(client->addInstance(plansys2::Instance("kitchen", "room")));
}
