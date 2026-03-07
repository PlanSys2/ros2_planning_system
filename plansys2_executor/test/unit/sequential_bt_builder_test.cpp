// Copyright 2025 Intelligent Robotics Lab
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

#include "plansys2_executor/bt_builder_plugins/sequential_bt_builder.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_msgs/srv/validate_domain.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include "plansys2_core/Utils.hpp"

#include "rclcpp/rclcpp.hpp"


class ROS2Environment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};


class SequentialBTBuilderTest : public plansys2::SequentialBTBuilder
{
public:
  SequentialBTBuilderTest()
  : SequentialBTBuilder() {initialize();}

  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan)
  {
    return plansys2::SequentialBTBuilder::get_tree(current_plan);
  }
};

class TestPlannerNode : public rclcpp::Node
{
private:
  rclcpp::Service<plansys2_msgs::srv::ValidateDomain>::SharedPtr validate_domain_service_ =
    create_service<plansys2_msgs::srv::ValidateDomain>(
    "planner/validate_domain",
    std::bind(
      &TestPlannerNode::validate_domain_service_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

public:
  TestPlannerNode()
  : Node("test_planner_node") {}

  void validate_domain_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Response> response)
  {
    response->success = true;
  }
};

TEST(sequential_btbuilder_tests, test_plan_with_derived_existential)
{
  {
    auto test_node = rclcpp::Node::make_shared("test_plan_1");
    auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
    auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
    auto planner_node = std::make_shared<TestPlannerNode>();

    auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
    auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

    auto btbuilder = std::make_shared<SequentialBTBuilderTest>();

    std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

    domain_node->set_parameter({"model_file", pkgpath + "/pddl/suave_domain.pddl"});
    domain_node->set_parameter({"validate_using_planner_node", true});
    problem_node->set_parameter({"model_file", pkgpath + "/pddl/suave_domain.pddl"});
    problem_node->set_parameter({"problem_file", pkgpath + "/pddl/suave_problem.pddl"});

    rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

    exe.add_node(domain_node->get_node_base_interface());
    exe.add_node(problem_node->get_node_base_interface());
    exe.add_node(planner_node->get_node_base_interface());

    bool finish = false;
    std::thread t([&]() {
        while (!finish) {
          exe.spin_some();
        }
      });

    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    plansys2_msgs::msg::Plan plan;

    plansys2_msgs::msg::PlanItem plan_item_0;
    plansys2_msgs::msg::PlanItem plan_item_1;
    plansys2_msgs::msg::PlanItem plan_item_2;
    plansys2_msgs::msg::PlanItem plan_item_3;
    plansys2_msgs::msg::PlanItem plan_item_4;
    plansys2_msgs::msg::PlanItem plan_item_5;
    plansys2_msgs::msg::PlanItem plan_item_6;
    plan_item_0.action = "(start_robot bluerov)";
    plan_item_1.action = "(reconfigure1 f_maintain_motion fd_all_thrusters)";
    plan_item_2.action = "(reconfigure1 f_generate_search_path fd_spiral_high)";
    plan_item_3.action = "(search_pipeline pipeline bluerov)";
    plan_item_4.action = "(reconfigure1 f_follow_pipeline fd_follow_pipeline)";
    plan_item_5.action = "(reconfigure2 f_generate_search_path fd_spiral_high fd_unground)";
    plan_item_6.action = "(inspect_pipeline pipeline bluerov)";

    plan.items.push_back(plan_item_0);
    plan.items.push_back(plan_item_1);
    plan.items.push_back(plan_item_2);
    plan.items.push_back(plan_item_3);
    plan.items.push_back(plan_item_4);
    plan.items.push_back(plan_item_5);
    plan.items.push_back(plan_item_6);

    auto bt_string = btbuilder->get_tree(plan);

    const std::string expected_bt_string =
      R"(<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="MainSequence">
      <Sequence name="(start_robot bluerov):0">
        <ApplyAtStartEffect action="(start_robot bluerov):0"/>
        <ReactiveSequence name="(start_robot bluerov):0">
            <CheckOverAllReq action="(start_robot bluerov):0"/>
            <ExecuteAction action="(start_robot bluerov):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(start_robot bluerov):0"/>
        <ApplyAtEndEffect action="(start_robot bluerov):0"/>
      </Sequence>
      <Sequence name="(reconfigure1 f_maintain_motion fd_all_thrusters):0">
        <WaitAction action="(start_robot bluerov):0"/>
        <ApplyAtStartEffect action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <ReactiveSequence name="(reconfigure1 f_maintain_motion fd_all_thrusters):0">
            <CheckOverAllReq action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
            <ExecuteAction action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <ApplyAtEndEffect action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
      </Sequence>
      <Sequence name="(reconfigure1 f_generate_search_path fd_spiral_high):0">
        <WaitAction action="(start_robot bluerov):0"/>
        <WaitAction action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <ApplyAtStartEffect action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        <ReactiveSequence name="(reconfigure1 f_generate_search_path fd_spiral_high):0">
            <CheckOverAllReq action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
            <ExecuteAction action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        <ApplyAtEndEffect action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
      </Sequence>
      <Sequence name="(search_pipeline pipeline bluerov):0">
        <WaitAction action="(start_robot bluerov):0"/>
        <WaitAction action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <WaitAction action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        <ApplyAtStartEffect action="(search_pipeline pipeline bluerov):0"/>
        <ReactiveSequence name="(search_pipeline pipeline bluerov):0">
            <CheckOverAllReq action="(search_pipeline pipeline bluerov):0"/>
            <ExecuteAction action="(search_pipeline pipeline bluerov):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(search_pipeline pipeline bluerov):0"/>
        <ApplyAtEndEffect action="(search_pipeline pipeline bluerov):0"/>
      </Sequence>
      <Sequence name="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0">
        <WaitAction action="(start_robot bluerov):0"/>
        <WaitAction action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <WaitAction action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        <WaitAction action="(search_pipeline pipeline bluerov):0"/>
        <ApplyAtStartEffect action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
        <ReactiveSequence name="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0">
            <CheckOverAllReq action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
            <ExecuteAction action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
        <ApplyAtEndEffect action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
      </Sequence>
      <Sequence name="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0">
        <WaitAction action="(start_robot bluerov):0"/>
        <WaitAction action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <WaitAction action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        <WaitAction action="(search_pipeline pipeline bluerov):0"/>
        <WaitAction action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
        <ApplyAtStartEffect action="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0"/>
        <ReactiveSequence name="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0">
            <CheckOverAllReq action="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0"/>
            <ExecuteAction action="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0"/>
        <ApplyAtEndEffect action="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0"/>
      </Sequence>
      <Sequence name="(inspect_pipeline pipeline bluerov):0">
        <WaitAction action="(start_robot bluerov):0"/>
        <WaitAction action="(reconfigure1 f_maintain_motion fd_all_thrusters):0"/>
        <WaitAction action="(reconfigure1 f_generate_search_path fd_spiral_high):0"/>
        <WaitAction action="(search_pipeline pipeline bluerov):0"/>
        <WaitAction action="(reconfigure1 f_follow_pipeline fd_follow_pipeline):0"/>
        <WaitAction action="(reconfigure2 f_generate_search_path fd_spiral_high fd_unground):0"/>
        <ApplyAtStartEffect action="(inspect_pipeline pipeline bluerov):0"/>
        <ReactiveSequence name="(inspect_pipeline pipeline bluerov):0">
            <CheckOverAllReq action="(inspect_pipeline pipeline bluerov):0"/>
            <ExecuteAction action="(inspect_pipeline pipeline bluerov):0"/>
        </ReactiveSequence>
        <CheckAtEndReq action="(inspect_pipeline pipeline bluerov):0"/>
        <ApplyAtEndEffect action="(inspect_pipeline pipeline bluerov):0"/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
)";

    EXPECT_EQ(bt_string, expected_bt_string);

    finish = true;
    t.join();
}
  plansys2::drain_ros(200ms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new ROS2Environment);

  return RUN_ALL_TESTS();
}
