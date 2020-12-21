// Copyright 2020 Intelligent Robotics Lab
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

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "plansys2_terminal/Terminal.hpp"

#include "gtest/gtest.h"

TEST(terminal_test, token_utils)
{
  auto test1 = plansys2_terminal::tokenize("");
  ASSERT_EQ(test1.size(), 0u);

  auto test2 = plansys2_terminal::tokenize("set");
  ASSERT_EQ(test2.size(), 1u);
  ASSERT_EQ(test2[0], "set");

  auto test3 = plansys2_terminal::tokenize("set domain");
  ASSERT_EQ(test3.size(), 2u);
  ASSERT_EQ(test3[0], "set");
  ASSERT_EQ(test3[1], "domain");

  auto test4 = plansys2_terminal::tokenize("set domain get");
  ASSERT_EQ(test4.size(), 3u);
  ASSERT_EQ(test4[0], "set");
  ASSERT_EQ(test4[1], "domain");
  ASSERT_EQ(test4[2], "get");

  plansys2_terminal::pop_front(test4);
  ASSERT_EQ(test4.size(), 2u);
  ASSERT_EQ(test4[0], "domain");
  ASSERT_EQ(test4[1], "get");

  plansys2_terminal::pop_front(test1);
  ASSERT_EQ(test1.size(), 0u);
}

class TerminalTest : public plansys2_terminal::Terminal
{
public:
  void init()
  {
    method_executed_["init"] = true;
    Terminal::init();
  }

  void clean_command(std::string & command)
  {
    method_executed_["clean_command"] = true;
    Terminal::clean_command(command);
  }

  void process_get_model_predicate(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_get_model_predicate"] = true;
    Terminal::process_get_model_predicate(command, os);
  }

  void process_get_model_action(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_get_model_action"] = true;
    Terminal::process_get_model_action(command, os);
  }

  void process_get_model(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_get_model"] = true;
    Terminal::process_get_model(command, os);
  }

  void process_get_problem(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_get_problem"] = true;
    Terminal::process_get_problem(command, os);
  }

  void process_get(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_get"] = true;
    Terminal::process_get(command, os);
  }

  void process_set_instance(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_set_instance"] = true;
    Terminal::process_set_instance(command, os);
  }

  void process_set_assignment(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_set_assignment"] = true;
    Terminal::process_set_assignment(command, os);
  }

  void process_set_predicate(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_set_predicate"] = true;
    Terminal::process_set_predicate(command, os);
  }

  void process_set_goal(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_set_goal"] = true;
    Terminal::process_set_goal(command, os);
  }

  void process_set(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_set"] = true;
    Terminal::process_set(command, os);
  }

  void process_remove_instance(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_remove_instance"] = true;
    Terminal::process_remove_instance(command, os);
  }

  void process_remove_predicate(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_remove_predicate"] = true;
    Terminal::process_remove_predicate(command, os);
  }

  void process_remove(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_remove"] = true;
    Terminal::process_remove(command, os);
  }

  void execute_plan()
  {
    method_executed_["execute_plan"] = true;
    Terminal::execute_plan();
  }

  void process_run(std::vector<std::string> & command, std::ostringstream & os)
  {
    method_executed_["process_run"] = true;
    Terminal::process_run(command, os);
  }

  void process_command(std::string & command, std::ostringstream & os)
  {
    method_executed_["process_command"] = true;
    Terminal::process_command(command, os);
  }

  void reset_executions()
  {
    for (auto it = method_executed_.begin(); it != method_executed_.end(); ++it) {
      it->second = false;
    }
  }

  std::map<std::string, bool> method_executed_;
};

TEST(terminal_test, load_popf_plugin)
{
  auto test_node = rclcpp::Node::make_shared("terminal_node_test");

  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>(test_node);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_terminal");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/simple_example.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/simple_example.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  auto terminal_node = std::make_shared<TerminalTest>();
  terminal_node->init();

  std::string command_1("    hey how        do   you    test  ");
  terminal_node->clean_command(command_1);
  ASSERT_EQ(command_1, "hey how do you test");

  {
    std::ostringstream os;
    std::string command("set instance leia robot");
    terminal_node->process_command(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_instance"]);
    ASSERT_TRUE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::string command("set instance r2d2 robot");
    terminal_node->process_command(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_instance"]);
    ASSERT_TRUE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"entrance", "room"};
    terminal_node->process_set_instance(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_instance"]);
    ASSERT_TRUE(os.str().empty());

    // auto ins_2 = problem_client->getInstance("r2d2");
    // ASSERT_TRUE(ins_2.has_value());
    // ASSERT_EQ(ins_2.value().name, "r2d2");
    // ASSERT_EQ(ins_2.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::string command("set predicate (robot_at leia entrance)");
    terminal_node->process_command(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_predicate"]);
    ASSERT_TRUE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::string command("set predicate (robot_at data voyager)");
    terminal_node->process_command(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_predicate"]);
    ASSERT_FALSE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"(robot_at", "r2d2", "entrance)"};
    terminal_node->process_set_predicate(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_predicate"]);
    ASSERT_TRUE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::string command("set goal (and(robot_at leia bathroom))");
    terminal_node->process_command(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_goal"]);
    ASSERT_FALSE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::string command("set goal (and(robot_at leia entrance))");
    terminal_node->process_command(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_set_goal"]);
    ASSERT_TRUE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"(=(robot_at", "r2d2", "entrance)", "0)"};
    terminal_node->process_set_assignment(command, os);
    // ASSERT_TRUE(terminal_node->method_executed_["process_command"]);
    // ASSERT_TRUE(terminal_node->method_executed_["process_set_goal"]);
    // ASSERT_TRUE(os.str().empty());

    // auto ins_1 = problem_client->getInstance("leia");
    // ASSERT_TRUE(ins_1.has_value());
    // ASSERT_EQ(ins_1.value().name, "leia");
    // ASSERT_EQ(ins_1.value().type, "robot");

    terminal_node->reset_executions();
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"(robot_at", "r2d2", "entrance)"};
    terminal_node->process_get_model_predicate(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_get_model_predicate"]);
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"move"};
    terminal_node->process_get_model_action(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_get_model_action"]);
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"types"};
    terminal_node->process_get_model(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_get_model"]);
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"instances"};
    terminal_node->process_get_problem(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_get_problem"]);
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"problem", "instances"};
    terminal_node->process_get(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_get"]);
    ASSERT_TRUE(terminal_node->method_executed_["process_get_problem"]);
  }

  {
    std::ostringstream os;
    std::vector<std::string> command = {"r2d2"};
    terminal_node->process_remove_instance(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_remove_instance"]);
    ASSERT_TRUE(os.str().empty());
  }

  {
    {
      std::ostringstream os;
      std::string command = "set instance leia robot";
      terminal_node->process_command(command, os);
    }
    {
      std::ostringstream os;
      std::string command = "set instance entrance room";
      terminal_node->process_command(command, os);
    }
    {
      std::ostringstream os;
      std::string command = "set instance kitchen room";
      terminal_node->process_command(command, os);
    }
    {
      std::ostringstream os;
      std::string command = "set predicate (robot_at leia entrance)";
      terminal_node->process_command(command, os);
    }
    {
      std::ostringstream os;
      std::string command = "set goal (and(robot_at leia kitchen))";
      terminal_node->process_command(command, os);
    }

    terminal_node->reset_executions();

    terminal_node->execute_plan();
    ASSERT_TRUE(terminal_node->method_executed_["execute_plan"]);

    terminal_node->reset_executions();

    std::ostringstream os;
    std::vector<std::string> command = {};
    terminal_node->process_run(command, os);
    ASSERT_TRUE(terminal_node->method_executed_["process_run"]);
  }

  finish = true;
  t.join();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
