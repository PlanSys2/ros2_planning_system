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

// STNBTBuilder on hand-written plans of the stn_test domain

#include <atomic>
#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "plansys2_core/PackagePaths.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/check_action_node.hpp"
#include "plansys2_executor/behavior_tree/stn_dependency.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/bt_builder_plugins/stn_bt_builder.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using plansys2::ActionType;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

class TestableSTN : public plansys2::STNBTBuilder
{
public:
  using STNBTBuilder::build_stn;
  using STNBTBuilder::can_apply;
  using STNBTBuilder::check_paths;
  using STNBTBuilder::floyd_warshall;
  using STNBTBuilder::from_state;
  using STNBTBuilder::get_conditions;
  using STNBTBuilder::get_diff;
  using STNBTBuilder::get_effects;
  using STNBTBuilder::get_happening;
  using STNBTBuilder::get_happenings;
  using STNBTBuilder::get_intersection;
  using STNBTBuilder::get_previous;
  using STNBTBuilder::get_simple_plan;
  using STNBTBuilder::get_states;
  using STNBTBuilder::is_executable;
  using STNBTBuilder::prune_paths;
};

plansys2_msgs::msg::Plan make_plan(
  const std::vector<std::tuple<double, std::string, double>> & items)
{
  plansys2_msgs::msg::Plan plan;
  for (const auto & [time, action, duration] : items) {
    plansys2_msgs::msg::PlanItem item;
    item.time = time;
    item.action = action;
    item.duration = duration;
    plan.items.push_back(item);
  }
  return plan;
}

}  // namespace

class STNBTBuilderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto pddl = plansys2::share_path("plansys2_executor").string() +
      "/pddl/stn_test.pddl";
    domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
    problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();
    domain_node_->set_parameter({"model_file", pddl});
    problem_node_->set_parameter({"model_file", pddl});
    exe_.add_node(domain_node_->get_node_base_interface());
    exe_.add_node(problem_node_->get_node_base_interface());
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
    for (auto node : {std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(domain_node_),
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(problem_node_)})
    {
      node->trigger_transition(Transition::TRANSITION_CONFIGURE);
      node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    }

    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    problem_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    for (auto [name, type] : std::vector<std::pair<std::string, std::string>>{
      {"r1", "robot"}, {"kitchen", "room"}, {"bedroom", "room"},
      {"m1", "match"}, {"m2", "match"}, {"f1", "fuse"}, {"f2", "fuse"}})
    {
      ASSERT_TRUE(problem_->addInstance(plansys2::Instance(name, type)));
    }
    for (auto predicate : {"(robot_at r1 kitchen)", "(connected kitchen bedroom)",
        "(handfree)", "(unused m1)", "(unused m2)"})
    {
      ASSERT_TRUE(problem_->addPredicate(plansys2::Predicate(predicate)));
    }
    ASSERT_TRUE(problem_->addFunction(plansys2::Function("(= (battery r1) 100)")));

    builder_ = std::make_shared<TestableSTN>();
    builder_->set_clients(std::make_shared<plansys2::DomainExpertClient>(client_node_), problem_);
    builder_->initialize("", "", 3);
  }

  void TearDown() override
  {
    builder_.reset();
    problem_.reset();
    spinner_.reset();
  }

  // As POPF solves MatchCellar: the fuse is mended while the match burns
  plansys2_msgs::msg::Plan matchcellar_plan()
  {
    return make_plan({{0.0, "(light_match m1)", 5.0}, {0.001, "(mend_fuse f1 m1)", 2.0}});
  }

  // Two fuses and a single free hand: the second mend waits for the first one to end
  plansys2_msgs::msg::Plan two_fuses_plan()
  {
    return make_plan({
      {0.0, "(light_match m1)", 5.0}, {0.001, "(mend_fuse f1 m1)", 2.0},
      {2.002, "(light_match m2)", 5.0}, {2.003, "(mend_fuse f2 m2)", 2.0}});
  }

  // A numeric condition and effect, then a non-durative action
  plansys2_msgs::msg::Plan move_plan()
  {
    return make_plan({
      {0.0, "(move r1 kitchen bedroom)", 5.0}, {5.001, "(switch_on r1 bedroom)", 0.001}});
  }

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
  rclcpp::Node::SharedPtr client_node_;  // The clients do not own it
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_;
  std::shared_ptr<TestableSTN> builder_;
};

namespace
{

bool has_predicate(const plansys2::StateVec & state, const std::string & predicate)
{
  for (const auto & p : state.predicates) {
    if (parser::pddl::toString(p) == predicate) {
      return true;
    }
  }
  return false;
}

plansys2::Node::Ptr find_node(
  const plansys2::Graph::Ptr & graph, const std::string & expression, ActionType type)
{
  for (const auto & node : graph->nodes) {
    if (node->action.expression == expression && node->action.type == type) {
      return node;
    }
  }
  return nullptr;
}

std::pair<double, double> arc(const plansys2::Node::Ptr & from, const plansys2::Node::Ptr & to)
{
  for (const auto & [node, lower, upper] : from->output_arcs) {
    if (node == to) {
      return {lower, upper};
    }
  }
  return {-1.0, -1.0};
}

}  // namespace

TEST_F(STNBTBuilderTest, default_templates_when_none_are_given)
{
  auto tree = builder_->get_tree(matchcellar_plan());
  for (const std::string node : {"WaitAtStartReq", "ApplyAtStartEffect", "CheckOverAllReq",
      "ExecuteAction", "CheckAtEndReq", "ApplyAtEndEffect"})
  {
    EXPECT_NE(tree.find("<" + node), std::string::npos) << node;
  }
}

TEST_F(STNBTBuilderTest, an_action_runs_during_another_one)
{
  auto tree = builder_->get_tree(matchcellar_plan());
  ASSERT_FALSE(tree.empty());

  // mend_fuse waits for the START of light_match, not for its end
  EXPECT_NE(
    tree.find("WaitAction action=\"(mend_fuse f1 m1):1 START (light_match m1):0 START\""),
    std::string::npos) << tree;
  // and the match is not put out before the fuse is mended
  EXPECT_NE(
    tree.find("CheckAction action=\"(light_match m1):0 END (mend_fuse f1 m1):1 END\""),
    std::string::npos) << tree;

  // The STN computes the window: mend_fuse (2 s) must start in the first 3 s of the match (5 s)
  auto graph = builder_->get_graph();
  auto light_start = find_node(graph, "(light_match m1)", ActionType::START);
  auto mend_start = find_node(graph, "(mend_fuse f1 m1)", ActionType::START);
  ASSERT_TRUE(light_start && mend_start);
  auto [lower, upper] = arc(light_start, mend_start);
  EXPECT_NEAR(lower, 0.0, 1e-6);
  EXPECT_NEAR(upper, 3.0, 1e-6);
}

TEST_F(STNBTBuilderTest, a_threat_orders_actions_of_different_happenings)
{
  // A single free hand: the second fuse is mended after the first one
  auto tree = builder_->get_tree(two_fuses_plan());
  ASSERT_FALSE(tree.empty());
  EXPECT_NE(
    tree.find("WaitAction action=\"(mend_fuse f2 m2):2003 START (mend_fuse f1 m1):1 END\""),
    std::string::npos) << tree;
  EXPECT_NE(
    tree.find("WaitAction action=\"(mend_fuse f2 m2):2003 START (light_match m2):2002 START\""),
    std::string::npos) << tree;
}

TEST_F(STNBTBuilderTest, numeric_conditions_and_non_durative_actions)
{
  auto tree = builder_->get_tree(move_plan());
  ASSERT_FALSE(tree.empty());
  EXPECT_NE(
    tree.find("WaitAction action=\"(switch_on r1 bedroom):5001 START "
    "(move r1 kitchen bedroom):0 END\""), std::string::npos) << tree;

  // The battery is part of the initial state
  auto init = builder_->from_state(problem_->getPredicates(), problem_->getFunctions());
  EXPECT_NE(parser::pddl::toString(init).find("battery r1"), std::string::npos);
}

TEST_F(STNBTBuilderTest, a_non_durative_action_with_no_duration)
{
  // Planners give non-durative actions a duration of 0, not only 0.001
  auto plan = make_plan({
    {0.0, "(move r1 kitchen bedroom)", 5.0}, {5.001, "(switch_on r1 bedroom)", 0.0}});
  EXPECT_TRUE(builder_->is_executable(plan));
  auto tree = builder_->get_tree(plan);
  ASSERT_FALSE(tree.empty());
  EXPECT_NE(tree.find("(switch_on r1 bedroom):5001"), std::string::npos) << tree;

  // Alone in the plan, as in a plan of a domain with no durative actions
  plan = make_plan({{0.0, "(switch_on r1 kitchen)", 0.0}});
  tree = builder_->get_tree(plan);
  ASSERT_FALSE(tree.empty());
  EXPECT_NE(tree.find("(switch_on r1 kitchen):0"), std::string::npos) << tree;
}

TEST_F(STNBTBuilderTest, a_plan_that_cannot_run_gets_no_tree)
{
  // Mending without a lit match
  auto plan = make_plan({{0.0, "(mend_fuse f1 m1)", 2.0}});
  EXPECT_FALSE(builder_->is_executable(plan));
  EXPECT_TRUE(builder_->get_tree(plan).empty());

  // Not enough battery to move
  ASSERT_TRUE(problem_->updateFunction(plansys2::Function("(= (battery r1) 5)")));
  EXPECT_FALSE(builder_->is_executable(move_plan()));
  EXPECT_TRUE(builder_->get_tree(move_plan()).empty());

  // Valid plans are
  EXPECT_TRUE(builder_->is_executable(matchcellar_plan()));
  EXPECT_TRUE(builder_->is_executable(two_fuses_plan()));
}

TEST_F(STNBTBuilderTest, happenings_of_a_plan)
{
  // Times with one more digit than the action ids: 0.001 s is 10
  auto happenings = builder_->get_happenings(matchcellar_plan());
  EXPECT_EQ(happenings, (std::set<int64_t>{-1, 0, 10, 20010, 50000}));

  // The happening at a time, or the last one before it
  EXPECT_EQ(*builder_->get_happening(10, happenings), 10);
  EXPECT_EQ(*builder_->get_happening(15, happenings), 10);
  EXPECT_EQ(*builder_->get_happening(-1, happenings), -1);
  EXPECT_EQ(builder_->get_happening(-5, happenings), happenings.end());
  EXPECT_EQ(builder_->get_happening(60000, happenings), happenings.end());

  EXPECT_EQ(*builder_->get_previous(10, happenings), 0);
  EXPECT_EQ(*builder_->get_previous(0, happenings), -1);
  EXPECT_EQ(builder_->get_previous(-1, happenings), happenings.end());
  EXPECT_EQ(builder_->get_previous(-5, happenings), happenings.end());
}

TEST_F(STNBTBuilderTest, times_are_rounded_and_long_plans_fit)
{
  // Truncated, 1.001 s was 1.000 s: 1.001 * 1000 is 1000.99999
  plansys2_msgs::msg::PlanItem item;
  item.action = "(light_match m1)";
  item.time = 1.001;
  EXPECT_EQ(plansys2::BTBuilder::to_action_id(item, 3), "(light_match m1):1001");
  item.time = 4.9999;
  EXPECT_EQ(plansys2::BTBuilder::to_action_id(item, 3), "(light_match m1):5000");
  EXPECT_EQ(plansys2::BTBuilder::to_action_id(item, 4), "(light_match m1):49999");
  EXPECT_EQ(plansys2::BTBuilder::to_int_time(-0.0004, 3), 0);

  // Three days in, in tenths of a ms: more than an int holds
  auto happenings = builder_->get_happenings(
    make_plan({{1.001, "(light_match m1)", 2.0005}, {259200.0007, "(mend_fuse f1 m1)", 2.0}}));
  EXPECT_EQ(
    happenings,
    (std::set<int64_t>{-1, 10010, 30015, 2592000007LL, 2592020007LL}));
}

TEST_F(STNBTBuilderTest, the_induced_simple_plan_and_its_states)
{
  auto plan = matchcellar_plan();
  auto happenings = builder_->get_happenings(plan);
  auto simple_plan = builder_->get_simple_plan(plan);

  std::map<ActionType, int> types;
  for (const auto & [time, action] : simple_plan) {
    types[action.type]++;
  }
  EXPECT_EQ(types[ActionType::INIT], 1);
  EXPECT_EQ(types[ActionType::START], 2);
  EXPECT_EQ(types[ActionType::END], 2);
  // mend_fuse needs the light all along: one check between each pair of its happenings
  EXPECT_GE(types[ActionType::OVERALL], 1);

  // The match is lit from its start to its end
  auto states = builder_->get_states(happenings, simple_plan);
  EXPECT_FALSE(has_predicate(states.at(-1), "(light m1)"));
  EXPECT_TRUE(has_predicate(states.at(0), "(light m1)"));
  EXPECT_TRUE(has_predicate(states.at(20010), "(mended f1)"));
  EXPECT_FALSE(has_predicate(states.at(50000), "(light m1)"));
}

TEST_F(STNBTBuilderTest, states_compare_predicates_and_functions)
{
  plansys2::StateVec a, b;
  a.predicates = {plansys2::Predicate("(handfree)"), plansys2::Predicate("(unused m1)")};
  b.predicates = {plansys2::Predicate("(handfree)"), plansys2::Predicate("(light m1)")};
  a.functions = {plansys2::Function("(= (battery r1) 100)")};
  b.functions = {plansys2::Function("(= (battery r1) 90)")};

  auto diff = builder_->get_diff(a, b);
  EXPECT_TRUE(has_predicate(diff, "(unused m1)"));
  EXPECT_FALSE(has_predicate(diff, "(handfree)"));
  EXPECT_EQ(diff.functions.size(), 1u);

  // Functions are compared by name and arguments: the battery is in both, its change is a diff
  auto common = builder_->get_intersection(a, b);
  EXPECT_TRUE(has_predicate(common, "(handfree)"));
  EXPECT_FALSE(has_predicate(common, "(unused m1)"));
  EXPECT_EQ(common.functions.size(), 1u);

  b.functions = a.functions;
  EXPECT_TRUE(builder_->get_diff(a, b).functions.empty());
  b.functions = {plansys2::Function("(= (battery r2) 100)")};
  EXPECT_TRUE(builder_->get_intersection(a, b).functions.empty());
}

TEST_F(STNBTBuilderTest, conditions_and_effects_of_each_snap_action)
{
  auto plan = matchcellar_plan();
  for (const auto & [time, action] : builder_->get_simple_plan(plan)) {
    auto conditions = parser::pddl::toString(builder_->get_conditions(action));
    auto effects = parser::pddl::toString(builder_->get_effects(action));
    if (action.expression == "(mend_fuse f1 m1)" && action.type == ActionType::OVERALL) {
      EXPECT_NE(conditions.find("light m1"), std::string::npos);
      EXPECT_TRUE(effects.empty());
    }
    if (action.expression == "(light_match m1)" && action.type == ActionType::START) {
      EXPECT_NE(effects.find("light m1"), std::string::npos);
    }
    if (action.type == ActionType::INIT) {
      EXPECT_TRUE(conditions.empty());
      EXPECT_NE(effects.find("handfree"), std::string::npos);
    }
  }
  plansys2::ActionStamped unknown;
  unknown.type = ActionType::UNKNOWN;
  EXPECT_TRUE(builder_->get_conditions(unknown).nodes.empty());
  EXPECT_TRUE(builder_->get_effects(unknown).nodes.empty());
}

TEST_F(STNBTBuilderTest, propagation_tightens_bounds_and_detects_inconsistency)
{
  // a -[2, 5]-> b -[1, 3]-> c, and a -[0, inf)-> c
  auto graph = plansys2::Graph::make_shared();
  auto a = plansys2::Node::make_shared(0);
  auto b = plansys2::Node::make_shared(1);
  auto c = plansys2::Node::make_shared(2);
  graph->nodes = {a, b, c};
  auto link = [](auto & from, auto & to, double lo, double up) {
      from->output_arcs.insert({to, lo, up});
      to->input_arcs.insert({from, lo, up});
    };
  link(a, b, 2.0, 5.0);
  link(b, c, 1.0, 3.0);
  link(a, c, 0.0, kInf);

  ASSERT_TRUE(builder_->propagate(graph));
  auto [lower, upper] = arc(a, c);
  EXPECT_NEAR(lower, 3.0, 1e-6);
  EXPECT_NEAR(upper, 8.0, 1e-6);

  // c must also come at most 2 s after a: impossible
  link(c, a, -2.0, kInf);
  EXPECT_FALSE(builder_->propagate(graph));
}

TEST_F(STNBTBuilderTest, all_pairs_shortest_paths)
{
  Eigen::MatrixXd dist(3, 3);
  dist << 0, 4, kInf,
    kInf, 0, 1,
    2, kInf, 0;
  builder_->floyd_warshall(dist);
  EXPECT_DOUBLE_EQ(dist(0, 2), 5.0);
  EXPECT_DOUBLE_EQ(dist(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(dist(2, 1), 6.0);
}

TEST_F(STNBTBuilderTest, paths_between_nodes)
{
  auto graph = builder_->build_stn(two_fuses_plan());
  auto mend1_end = find_node(graph, "(mend_fuse f1 m1)", ActionType::END);
  auto mend2_end = find_node(graph, "(mend_fuse f2 m2)", ActionType::END);
  auto light2_end = find_node(graph, "(light_match m2)", ActionType::END);
  ASSERT_TRUE(mend1_end && mend2_end && light2_end);

  // The second mend follows the first one; the first does not follow the second
  EXPECT_TRUE(builder_->check_paths(mend2_end, mend1_end));
  EXPECT_FALSE(builder_->check_paths(mend1_end, mend2_end));
  EXPECT_TRUE(builder_->check_paths(light2_end, mend2_end));

  // A direct arc that a longer path already implies is removed
  auto mend2_start = find_node(graph, "(mend_fuse f2 m2)", ActionType::START);
  mend1_end->output_arcs.insert({light2_end, 0.0, kInf});
  light2_end->input_arcs.insert({mend1_end, 0.0, kInf});
  builder_->prune_paths(light2_end, mend1_end);
  EXPECT_EQ(arc(mend1_end, light2_end).first, -1.0);
  EXPECT_TRUE(builder_->check_paths(light2_end, mend1_end));
  (void)mend2_start;
}

TEST_F(STNBTBuilderTest, dotgraph_shows_each_action_status)
{
  EXPECT_TRUE(builder_->get_dotgraph(nullptr).empty());  // No plan yet

  auto plan = two_fuses_plan();
  ASSERT_FALSE(builder_->get_tree(plan).empty());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("dotgraph_test");
  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  std::vector<std::pair<std::string, plansys2::ActionExecutor::Status>> statuses = {
    {"(light_match m1):0", plansys2::ActionExecutor::SUCCESS},
    {"(mend_fuse f1 m1):1", plansys2::ActionExecutor::FAILURE},
    {"(light_match m2):2002", plansys2::ActionExecutor::RUNNING},
    {"(mend_fuse f2 m2):2003", plansys2::ActionExecutor::CANCELLED}};
  for (const auto & [id, status] : statuses) {
    (*action_map)[id].action_executor =
      plansys2::ActionExecutor::make_shared(id.substr(0, id.find(':')), node);
    (*action_map)[id].action_executor->set_internal_status(status);
  }

  auto dot = builder_->get_dotgraph(action_map, true, true);
  EXPECT_EQ(dot.rfind("digraph plan {", 0), 0u);
  EXPECT_NE(dot.find("label = \"Legend\""), std::string::npos);
  for (auto color : {"seagreen2", "skyblue", "pink"}) {
    EXPECT_NE(dot.find(color), std::string::npos) << color;
  }

  // An action with no executor yet
  (*action_map)["(mend_fuse f2 m2):2003"].action_executor = nullptr;
  EXPECT_FALSE(builder_->get_dotgraph(action_map).empty());
}

// The BT nodes of the STN on the network of a plan, as the executor runs them
class STNNodesTest : public STNBTBuilderTest
{
protected:
  void SetUp() override
  {
    STNBTBuilderTest::SetUp();
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("stn_nodes_test");
    node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(node_);
    domain_ = std::make_shared<plansys2::DomainExpertClient>(node_);
    factory_.registerNodeType<plansys2::WaitAction>("WaitAction");
    factory_.registerNodeType<plansys2::CheckAction>("CheckAction");
    factory_.registerNodeType<plansys2::ApplyAtStartEffect>("ApplyAtStartEffect");
    factory_.registerNodeType<plansys2::ApplyAtEndEffect>("ApplyAtEndEffect");
  }

  void TearDown() override
  {
    action_map_.reset();
    domain_.reset();
    STNBTBuilderTest::TearDown();
  }

  // Builds the network of the plan and an action map as ExecutorNode does
  void load(const plansys2_msgs::msg::Plan & plan)
  {
    ASSERT_FALSE(builder_->get_tree(plan).empty());
    action_map_ = std::make_shared<plansys2::ActionMap>();
    auto & init = (*action_map_)[":0"];
    init.action_executor = plansys2::ActionExecutor::make_shared("(INIT)", node_);
    init.action_executor->set_internal_status(plansys2::ActionExecutor::SUCCESS);
    init.at_start_effects_applied = init.at_end_effects_applied = true;
    init.at_start_effects_applied_time = init.at_end_effects_applied_time = node_->now();
    for (const auto & item : plan.items) {
      auto id = plansys2::BTBuilder::to_action_id(item, 3);
      auto & info = (*action_map_)[id];
      info.action_executor = plansys2::ActionExecutor::make_shared(item.action, node_);
      auto name = plansys2::get_action_name(item.action);
      auto params = plansys2::get_action_params(item.action);
      if (item.duration > 0.01) {
        info.action_info = domain_->getDurativeAction(name, params);
      } else {
        info.action_info = domain_->getAction(name, params);
      }
      ASSERT_FALSE(info.action_info.is_empty()) << id;
    }
  }

  BT::NodeStatus tick(const std::string & bt_node, const std::string & action, bool relaxed)
  {
    auto blackboard = BT::Blackboard::create();
    blackboard->set("action_map", action_map_);
    blackboard->set("action_graph", builder_->get_graph());
    blackboard->set("node", node_);
    blackboard->set("problem_client", problem_);
    blackboard->set("bt_builder", std::static_pointer_cast<plansys2::BTBuilder>(builder_));
    blackboard->set("stn_relaxed_timing", relaxed);
    auto tree = factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\"><" + bt_node + " action=\"" +
      action + "\"/></BehaviorTree></root>", blackboard);
    return tree.tickOnce();
  }

  // The event happened `ago` seconds before now
  void start(const std::string & id, double ago = 0.0)
  {
    (*action_map_)[id].at_start_effects_applied = true;
    (*action_map_)[id].at_start_effects_applied_time = node_->now() -
      rclcpp::Duration::from_seconds(ago);
  }

  void end(const std::string & id, double ago = 0.0)
  {
    (*action_map_)[id].at_end_effects_applied = true;
    (*action_map_)[id].at_end_effects_applied_time = node_->now() -
      rclcpp::Duration::from_seconds(ago);
    (*action_map_)[id].action_executor->set_internal_status(plansys2::ActionExecutor::SUCCESS);
  }

  // Sets the bounds of the arc between two events of the network
  void set_arc(
    const std::string & parent, ActionType parent_type, const std::string & child,
    ActionType child_type, double lower, double upper)
  {
    auto graph = builder_->get_graph();
    auto from = find_node(graph, parent, parent_type);
    auto to = find_node(graph, child, child_type);
    ASSERT_TRUE(from && to);
    for (auto it = from->output_arcs.begin(); it != from->output_arcs.end(); ++it) {
      if (std::get<0>(*it) == to) {
        from->output_arcs.erase(it);
        break;
      }
    }
    from->output_arcs.insert({to, lower, upper});
  }

  std::pair<double, double> bounds(
    const std::string & parent, ActionType parent_type, const std::string & child,
    ActionType child_type)
  {
    auto graph = builder_->get_graph();
    return arc(find_node(graph, parent, parent_type), find_node(graph, child, child_type));
  }

  static constexpr const char * kLight = "(light_match m1):0";
  static constexpr const char * kMend = "(mend_fuse f1 m1):1";
  static constexpr const char * kWaitMend = "(mend_fuse f1 m1):1 START (light_match m1):0 START";
  static constexpr const char * kLightAfterMend = "(light_match m1):0 END (mend_fuse f1 m1):1 END";

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<plansys2::testing::NodeSpinner> node_spinner_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_;
  std::shared_ptr<plansys2::ActionMap> action_map_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(STNNodesTest, wait_action_waits_for_the_parent_event)
{
  load(matchcellar_plan());
  for (bool relaxed : {true, false}) {
    (*action_map_)[kLight].at_start_effects_applied = false;
    EXPECT_EQ(tick("WaitAction", kWaitMend, relaxed), BT::NodeStatus::RUNNING) << relaxed;
    start(kLight);
    EXPECT_EQ(tick("WaitAction", kWaitMend, relaxed), BT::NodeStatus::SUCCESS) << relaxed;
  }

  // The first action only waits for INIT, and an action's end for its own start
  EXPECT_EQ(tick("WaitAction", "(light_match m1):0 START :0 INIT", true), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    tick("WaitAction", "(mend_fuse f1 m1):1 END (mend_fuse f1 m1):1 START", false),
    BT::NodeStatus::SUCCESS);

  // A parent with no executor has not happened
  (*action_map_)[kLight].action_executor = nullptr;
  EXPECT_EQ(tick("WaitAction", kWaitMend, true), BT::NodeStatus::RUNNING);
  (*action_map_).erase(kLight);
  EXPECT_EQ(tick("WaitAction", kWaitMend, true), BT::NodeStatus::RUNNING);
}

TEST_F(STNNodesTest, wait_action_keeps_the_planned_times_only_when_strict)
{
  load(matchcellar_plan());
  set_arc("(light_match m1)", ActionType::START, "(mend_fuse f1 m1)", ActionType::START, 1.0, 2.0);

  // Too early: strict waits, relaxed goes on
  start(kLight, 0.5);
  EXPECT_EQ(tick("WaitAction", kWaitMend, false), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tick("WaitAction", kWaitMend, true), BT::NodeStatus::SUCCESS);

  // Within the window and late: both go on, late only warns
  for (double ago : {1.0, 1.5, 2.0, 5.0}) {
    start(kLight, ago);
    EXPECT_EQ(tick("WaitAction", kWaitMend, false), BT::NodeStatus::SUCCESS) << ago;
    EXPECT_EQ(tick("WaitAction", kWaitMend, true), BT::NodeStatus::SUCCESS) << ago;
  }
}

TEST_F(STNNodesTest, wait_action_with_plain_ids_waits_for_the_action_to_finish)
{
  // SimpleBTBuilder writes plain ids and the blackboard has no network nor node
  load(matchcellar_plan());
  auto blackboard = BT::Blackboard::create();
  blackboard->set("action_map", action_map_);
  auto status = [&](const std::string & action) {
      return factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\"><WaitAction action=\"" +
        action + "\"/></BehaviorTree></root>", blackboard).tickOnce();
    };

  EXPECT_EQ(status("(unknown):5"), BT::NodeStatus::RUNNING);
  EXPECT_EQ(status(kLight), BT::NodeStatus::RUNNING);
  (*action_map_)[kLight].action_executor->set_internal_status(plansys2::ActionExecutor::SUCCESS);
  EXPECT_EQ(status(kLight), BT::NodeStatus::SUCCESS);
  (*action_map_)[kLight].action_executor = nullptr;
  EXPECT_EQ(status(kLight), BT::NodeStatus::RUNNING);

  // An STN dependency without a network nor a node: unbounded, with the ROS clock
  start(kLight);
  EXPECT_EQ(status(kWaitMend), BT::NodeStatus::RUNNING);  // No executor
  (*action_map_)[kLight].action_executor =
    plansys2::ActionExecutor::make_shared("(light_match m1)", node_);
  EXPECT_EQ(status(kWaitMend), BT::NodeStatus::SUCCESS);
}

TEST_F(STNNodesTest, check_action_checks_the_order_of_events)
{
  load(matchcellar_plan());
  for (bool relaxed : {true, false}) {
    // Only the STN builder uses CheckAction
    EXPECT_EQ(tick("CheckAction", kLight, relaxed), BT::NodeStatus::FAILURE);

    // light_match ends after mend_fuse ends
    EXPECT_EQ(tick("CheckAction", kLightAfterMend, relaxed), BT::NodeStatus::RUNNING);
    end(kMend);
    EXPECT_EQ(tick("CheckAction", kLightAfterMend, relaxed), BT::NodeStatus::SUCCESS);
    (*action_map_)[kMend].action_executor->set_internal_status(plansys2::ActionExecutor::RUNNING);
    EXPECT_EQ(tick("CheckAction", kLightAfterMend, relaxed), BT::NodeStatus::RUNNING);
    (*action_map_)[kMend].at_end_effects_applied = false;

    // An action's end follows its own start
    std::string own_end = std::string(kLight) + " END " + kLight + " START";
    (*action_map_)[kLight].at_start_effects_applied = false;
    EXPECT_EQ(tick("CheckAction", own_end, relaxed), BT::NodeStatus::RUNNING);
    start(kLight);
    EXPECT_EQ(tick("CheckAction", own_end, relaxed), BT::NodeStatus::SUCCESS);
  }
}

TEST_F(STNNodesTest, check_action_only_warns_out_of_its_window)
{
  load(matchcellar_plan());
  set_arc("(mend_fuse f1 m1)", ActionType::END, "(light_match m1)", ActionType::END, 1.0, 2.0);
  for (double ago : {0.0, 1.5, 3.0}) {
    end(kMend, ago);
    EXPECT_EQ(tick("CheckAction", kLightAfterMend, false), BT::NodeStatus::SUCCESS) << ago;
    EXPECT_EQ(tick("CheckAction", kLightAfterMend, true), BT::NodeStatus::SUCCESS) << ago;
  }
}

TEST_F(STNNodesTest, apply_effects_once_and_record_their_time)
{
  load(matchcellar_plan());
  ASSERT_EQ(tick("ApplyAtStartEffect", kLight, true), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(problem_->existPredicate(plansys2::Predicate("(light m1)")));
  EXPECT_FALSE(problem_->existPredicate(plansys2::Predicate("(unused m1)")));
  auto & light = (*action_map_)[kLight];
  EXPECT_TRUE(light.at_start_effects_applied);
  auto started = light.at_start_effects_applied_time;

  // A second tick applies nothing again
  ASSERT_TRUE(problem_->addPredicate(plansys2::Predicate("(unused m1)")));
  EXPECT_EQ(tick("ApplyAtStartEffect", kLight, true), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(problem_->existPredicate(plansys2::Predicate("(unused m1)")));
  EXPECT_EQ(light.at_start_effects_applied_time, started);

  ASSERT_EQ(tick("ApplyAtEndEffect", kMend, true), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(problem_->existPredicate(plansys2::Predicate("(mended f1)")));
  auto & mend = (*action_map_)[kMend];
  EXPECT_TRUE(mend.at_end_effects_applied);
  auto ended = mend.at_end_effects_applied_time;
  ASSERT_TRUE(problem_->removePredicate(plansys2::Predicate("(mended f1)")));
  EXPECT_EQ(tick("ApplyAtEndEffect", kMend, true), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(problem_->existPredicate(plansys2::Predicate("(mended f1)")));
  EXPECT_EQ(mend.at_end_effects_applied_time, ended);
}

TEST_F(STNNodesTest, the_start_of_a_non_durative_action_applies_nothing)
{
  load(move_plan());
  const std::string switch_on = "(switch_on r1 bedroom):5001";
  ASSERT_EQ(tick("ApplyAtStartEffect", switch_on, false), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE((*action_map_)[switch_on].at_start_effects_applied);
  EXPECT_FALSE(problem_->existPredicate(plansys2::Predicate("(light_on bedroom)")));
  ASSERT_EQ(tick("ApplyAtEndEffect", switch_on, false), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(problem_->existPredicate(plansys2::Predicate("(light_on bedroom)")));
}

TEST_F(STNNodesTest, strict_timing_fixes_the_network_to_the_actual_times)
{
  load(matchcellar_plan());
  start(kLight, 1.5);
  ASSERT_EQ(tick("ApplyAtStartEffect", kMend, false), BT::NodeStatus::SUCCESS);
  auto [lower, upper] =
    bounds("(light_match m1)", ActionType::START, "(mend_fuse f1 m1)", ActionType::START);
  EXPECT_NEAR(lower, 1.5, 0.1);
  EXPECT_DOUBLE_EQ(lower, upper);

  // Relaxed timing leaves the network as planned
  auto before = bounds("(mend_fuse f1 m1)", ActionType::START, "(mend_fuse f1 m1)",
    ActionType::END);
  ASSERT_EQ(tick("ApplyAtEndEffect", kMend, true), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    bounds("(mend_fuse f1 m1)", ActionType::START, "(mend_fuse f1 m1)", ActionType::END), before);
}

TEST_F(STNNodesTest, an_inconsistent_network_keeps_the_order)
{
  // light_match ends 0.1 s after its start, but mend_fuse needs 2 s inside it
  load(matchcellar_plan());
  start(kLight, 0.1);
  ASSERT_EQ(tick("ApplyAtEndEffect", kLight, false), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE((*action_map_)[kLight].at_end_effects_applied);
  EXPECT_FALSE(problem_->existPredicate(plansys2::Predicate("(light m1)")));
  EXPECT_FALSE(builder_->propagate(builder_->get_graph()));
}

// Found measuring long plans: each action of a sequence nested the next one, and past some
// hundreds of actions the tree was deeper than BT.CPP reads (XML_ELEMENT_DEPTH_EXCEEDED). The
// tree of a long sequence is now flat
TEST_F(STNBTBuilderTest, a_long_sequence_gives_a_flat_tree)
{
  // Nested, their tree had two levels per move; few, for slow coverage builds
  constexpr int kMoves = 40;
  for (int i = 0; i <= kMoves; i++) {
    ASSERT_TRUE(problem_->addInstance(plansys2::Instance("z" + std::to_string(i), "room")));
  }
  ASSERT_TRUE(problem_->addPredicate(plansys2::Predicate("(robot_at r1 z0)")));
  ASSERT_TRUE(problem_->removePredicate(plansys2::Predicate("(robot_at r1 kitchen)")));
  // Each move takes 10 of battery
  ASSERT_TRUE(problem_->updateFunction(plansys2::Function("(= (battery r1) 100000)")));
  std::vector<std::tuple<double, std::string, double>> items;
  for (int i = 0; i < kMoves; i++) {
    const auto from = "z" + std::to_string(i);
    const auto to = "z" + std::to_string(i + 1);
    ASSERT_TRUE(problem_->addPredicate(plansys2::Predicate("(connected " + from + " " + to + ")")));
    items.emplace_back(5.001 * i, "(move r1 " + from + " " + to + ")", 5.0);
  }

  const auto tree = builder_->get_tree(make_plan(items));
  ASSERT_FALSE(tree.empty());

  // How deep the XML is: it does not grow with the plan
  int depth = 0;
  int deepest = 0;
  for (size_t i = 0; i + 1 < tree.size(); i++) {
    if (tree[i] == '<' && tree[i + 1] == '/') {
      depth--;
    } else if (tree[i] == '<') {
      const auto close = tree.find('>', i);
      if (close != std::string::npos && tree[close - 1] != '/') {
        deepest = std::max(deepest, ++depth);
      }
    }
  }
  EXPECT_LT(deepest, 30) << "XML depth";
  // Every move is there
  for (int i = 0; i < kMoves; i++) {
    const auto move = "(move r1 z" + std::to_string(i) + " z" + std::to_string(i + 1) + ")";
    EXPECT_NE(tree.find(move), std::string::npos) << move;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
