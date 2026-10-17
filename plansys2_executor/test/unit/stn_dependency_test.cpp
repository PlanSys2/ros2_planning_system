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

// Helpers the BT nodes use to follow the STN of STNBTBuilder

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_executor/behavior_tree/stn_dependency.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// Propagation is the builder's business: count the calls and answer what the test says
class FakeBuilder : public plansys2::BTBuilder
{
public:
  void initialize(const std::string &, const std::string &, int) override {}
  std::string get_tree(const plansys2_msgs::msg::Plan &) override {return "";}
  plansys2::Graph::Ptr get_graph() override {return nullptr;}
  bool propagate(plansys2::Graph::Ptr) override {propagations++; return consistent;}
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, plansys2::ActionExecutionInfo>>, bool, bool) override
  {
    return "";
  }

  int propagations {0};
  bool consistent {true};
};

plansys2::Node::Ptr make_node(
  int num, const std::string & expression, double time, plansys2::ActionType type)
{
  auto node = plansys2::Node::make_shared(num);
  node->action.expression = expression;
  node->action.time = time;
  node->action.type = type;
  return node;
}

void link(const plansys2::Node::Ptr & from, const plansys2::Node::Ptr & to, double lo, double up)
{
  from->output_arcs.insert({to, lo, up});
  to->input_arcs.insert({from, lo, up});
}

const char kMove[] = "(move r2d2 a b):0";
const char kPick[] = "(pick r2d2 box b):5001";

}  // namespace

class StnDependencyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("stn_dependency_test");
    action_map_[":0"].action_executor = plansys2::ActionExecutor::make_shared("(INIT)", node_);
    action_map_[":0"].action_executor->set_internal_status(plansys2::ActionExecutor::SUCCESS);
    action_map_[":0"].at_end_effects_applied_time = rclcpp::Time(100, 0, RCL_ROS_TIME);
    for (const std::string id : {kMove, kPick}) {
      action_map_[id].action_executor =
        plansys2::ActionExecutor::make_shared(id.substr(0, id.find(':')), node_);
    }
  }

  rclcpp::Time at(double seconds)
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9),
             RCL_ROS_TIME);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  plansys2::ActionMap action_map_;
};

TEST(StnDependencyParse, the_four_parts_of_a_dependency)
{
  auto dependency = plansys2::parse_stn_dependency(
    "(pick r2d2 box b):5001 START (move r2d2 a b):0 END");
  ASSERT_TRUE(dependency);
  EXPECT_EQ(dependency->child_id, "(pick r2d2 box b):5001");
  EXPECT_EQ(dependency->child_type, "START");
  EXPECT_EQ(dependency->parent_id, "(move r2d2 a b):0");
  EXPECT_EQ(dependency->parent_type, "END");

  // The INIT pseudo-action, and an action's end after its own start
  dependency = plansys2::parse_stn_dependency("(move r2d2 a b):0 START :0 INIT");
  ASSERT_TRUE(dependency);
  EXPECT_EQ(dependency->parent_id, ":0");
  EXPECT_EQ(dependency->parent_type, "INIT");
  dependency = plansys2::parse_stn_dependency("(move r2d2 a b):0 END (move r2d2 a b):0 START");
  ASSERT_TRUE(dependency);
  EXPECT_EQ(dependency->child_type, "END");
  EXPECT_EQ(dependency->child_id, dependency->parent_id);
}

TEST(StnDependencyParse, plain_ids_and_garbage_are_not_dependencies)
{
  // What SimpleBTBuilder writes: a plain action id
  EXPECT_FALSE(plansys2::parse_stn_dependency("(move r2d2 a b):0"));
  EXPECT_FALSE(plansys2::parse_stn_dependency("(move r2d2 a b):5001"));
  for (const std::string text : {
    "", "START", "(move r2d2 a b):0 START", "(move r2d2 a b):0 STOP (x):0 END",
    "(move r2d2 a b):0 START (x):0 FINISH", "(move r2d2 a b) START (x):0 END",
    "(move r2d2 a b):0 START (x) END", "(move r2d2 a b):0  START (x):0 END"})
  {
    EXPECT_FALSE(plansys2::parse_stn_dependency(text)) << text;
  }
}

TEST(StnDependencyGraph, nodes_are_found_by_action_id_and_type)
{
  auto graph = plansys2::Graph::make_shared();
  auto move_start = make_node(1, "(move r2d2 a b)", 0.0, plansys2::ActionType::START);
  auto move_end = make_node(2, "(move r2d2 a b)", 0.0, plansys2::ActionType::END);
  graph->nodes = {move_start, move_end};

  EXPECT_EQ(plansys2::find_stn_node(graph, kMove, "START"), move_start);
  EXPECT_EQ(plansys2::find_stn_node(graph, kMove, "END"), move_end);
  EXPECT_EQ(plansys2::find_stn_node(graph, kMove, "OVERALL"), nullptr);
  EXPECT_EQ(plansys2::find_stn_node(graph, kPick, "START"), nullptr);
  EXPECT_EQ(plansys2::find_stn_node(nullptr, kMove, "START"), nullptr);
}

TEST(StnDependencyGraph, bounds_come_from_the_arc_between_both_events)
{
  auto graph = plansys2::Graph::make_shared();
  auto move_end = make_node(1, "(move r2d2 a b)", 0.0, plansys2::ActionType::END);
  auto pick_start = make_node(2, "(pick r2d2 box b)", 5.001, plansys2::ActionType::START);
  graph->nodes = {move_end, pick_start};
  link(move_end, pick_start, 1.5, 4.0);

  plansys2::StnDependency dependency{kPick, "START", kMove, "END"};
  auto [lower, upper] = plansys2::stn_bounds(graph, dependency);
  EXPECT_DOUBLE_EQ(lower, 1.5);
  EXPECT_DOUBLE_EQ(upper, 4.0);

  // No arc, unknown events or no graph at all: unbounded
  auto unbounded = std::make_pair(0.0, kInf);
  EXPECT_EQ(plansys2::stn_bounds(graph, {kMove, "END", kPick, "START"}), unbounded);
  EXPECT_EQ(plansys2::stn_bounds(graph, {kPick, "END", kMove, "END"}), unbounded);
  EXPECT_EQ(plansys2::stn_bounds(nullptr, dependency), unbounded);
}

TEST_F(StnDependencyTest, time_since_the_parent_event)
{
  plansys2::StnDependency after_init{kMove, "START", ":0", "INIT"};
  plansys2::StnDependency after_move_start{kPick, "START", kMove, "START"};
  plansys2::StnDependency after_move_end{kPick, "START", kMove, "END"};

  // INIT happened when the plan started
  auto elapsed = plansys2::seconds_since_parent(action_map_, after_init, at(102.5));
  ASSERT_TRUE(elapsed);
  EXPECT_NEAR(elapsed.value(), 2.5, 1e-6);

  // Nothing of the move has happened yet
  EXPECT_FALSE(plansys2::seconds_since_parent(action_map_, after_move_start, at(103)));
  EXPECT_FALSE(plansys2::seconds_since_parent(action_map_, after_move_end, at(103)));

  // Its start
  action_map_[kMove].at_start_effects_applied = true;
  action_map_[kMove].at_start_effects_applied_time = at(103);
  elapsed = plansys2::seconds_since_parent(action_map_, after_move_start, at(104));
  ASSERT_TRUE(elapsed);
  EXPECT_NEAR(elapsed.value(), 1.0, 1e-6);

  // Its end counts once its effects are applied and the action has finished
  action_map_[kMove].at_end_effects_applied = true;
  action_map_[kMove].at_end_effects_applied_time = at(106);
  EXPECT_FALSE(plansys2::seconds_since_parent(action_map_, after_move_end, at(107)));
  action_map_[kMove].action_executor->set_internal_status(plansys2::ActionExecutor::SUCCESS);
  elapsed = plansys2::seconds_since_parent(action_map_, after_move_end, at(107));
  ASSERT_TRUE(elapsed);
  EXPECT_NEAR(elapsed.value(), 1.0, 1e-6);

  // Unknown parent, or one with no executor yet
  EXPECT_FALSE(
    plansys2::seconds_since_parent(action_map_, {kPick, "START", "(x):1", "END"}, at(107)));
  action_map_[kPick].action_executor = nullptr;
  EXPECT_FALSE(
    plansys2::seconds_since_parent(action_map_, {kMove, "END", kPick, "START"}, at(107)));
}

TEST_F(StnDependencyTest, times_of_different_clock_types_do_not_throw)
{
  // A default rclcpp::Time is system time; the node clock is ROS time
  action_map_[kMove].at_start_effects_applied = true;
  action_map_[kMove].at_start_effects_applied_time = rclcpp::Time(103, 0);
  EXPECT_NO_THROW(
    plansys2::seconds_since_parent(action_map_, {kPick, "START", kMove, "START"}, at(104)));
}

TEST_F(StnDependencyTest, an_event_fixes_its_arcs_to_the_actual_times)
{
  auto graph = plansys2::Graph::make_shared();
  auto init = make_node(0, "", 0.0, plansys2::ActionType::INIT);
  auto move_start = make_node(1, "(move r2d2 a b)", 0.0, plansys2::ActionType::START);
  auto move_end = make_node(2, "(move r2d2 a b)", 0.0, plansys2::ActionType::END);
  auto pick_start = make_node(3, "(pick r2d2 box b)", 5.001, plansys2::ActionType::START);
  graph->nodes = {init, move_start, move_end, pick_start};
  link(init, move_start, 0.0, kInf);
  link(move_start, move_end, 5.0, 5.0);
  link(move_end, pick_start, 0.0, kInf);
  link(move_start, pick_start, 0.0, kInf);

  auto builder = std::make_shared<FakeBuilder>();

  // The move starts 3 s after INIT
  action_map_[kMove].at_start_effects_applied = true;
  action_map_[kMove].at_start_effects_applied_time = at(103);
  ASSERT_TRUE(
    plansys2::update_stn_on_event(
      graph, builder, action_map_, kMove, plansys2::ActionType::START, at(103)));
  EXPECT_EQ(builder->propagations, 1);
  ASSERT_EQ(move_start->input_arcs.size(), 1u);
  auto [parent, lower, upper] = *move_start->input_arcs.begin();
  EXPECT_EQ(parent, init);
  EXPECT_NEAR(lower, 3.0, 1e-6);
  EXPECT_NEAR(upper, 3.0, 1e-6);
  // The parent side of the arc says the same
  ASSERT_EQ(init->output_arcs.size(), 1u);
  EXPECT_NEAR(std::get<1>(*init->output_arcs.begin()), 3.0, 1e-6);

  // The pick starts: only the arc from the move start is fixed, its end has not happened
  ASSERT_TRUE(
    plansys2::update_stn_on_event(
      graph, builder, action_map_, kPick, plansys2::ActionType::START, at(104)));
  for (const auto & [from, lo, up] : pick_start->input_arcs) {
    if (from == move_start) {
      EXPECT_NEAR(lo, 1.0, 1e-6);
      EXPECT_NEAR(up, 1.0, 1e-6);
    } else {
      EXPECT_EQ(from, move_end);
      EXPECT_DOUBLE_EQ(lo, 0.0);
      EXPECT_EQ(up, kInf);
    }
  }

  // An inconsistent network is reported
  builder->consistent = false;
  EXPECT_FALSE(
    plansys2::update_stn_on_event(
      graph, builder, action_map_, kMove, plansys2::ActionType::START, at(103)));
}

TEST_F(StnDependencyTest, without_an_stn_there_is_nothing_to_update)
{
  auto builder = std::make_shared<FakeBuilder>();
  auto graph = plansys2::Graph::make_shared();

  // SimpleBTBuilder plans: no graph, or events not in it
  EXPECT_TRUE(
    plansys2::update_stn_on_event(
      nullptr, builder, action_map_, kMove, plansys2::ActionType::START, at(1)));
  EXPECT_TRUE(
    plansys2::update_stn_on_event(
      graph, builder, action_map_, kMove, plansys2::ActionType::START, at(1)));
  graph->nodes = {make_node(1, "(move r2d2 a b)", 0.0, plansys2::ActionType::START)};
  EXPECT_TRUE(
    plansys2::update_stn_on_event(
      graph, nullptr, action_map_, kMove, plansys2::ActionType::START, at(1)));
  EXPECT_EQ(builder->propagations, 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
