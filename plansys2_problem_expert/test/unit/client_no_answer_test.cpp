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

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/srv/exist_node.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_core/TestUtils.hpp"
#include "plansys2_core/Utils.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using ExistNode = plansys2_msgs::srv::ExistNode;
using GetProblemInstances = plansys2_msgs::srv::GetProblemInstances;

// Problem expert that only answers what it knows: the rest of the requests never get a response
class PartialProblemExpert : public rclcpp::Node
{
public:
  PartialProblemExpert()
  : rclcpp::Node("problem_expert")
  {
    state_ = plansys2::testing::active_state(*this, "problem_expert");
    exist_predicate_ = create_service<ExistNode>(
      "problem_expert/exist_problem_predicate",
      [this](std::shared_ptr<rclcpp::Service<ExistNode>> service,
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<ExistNode::Request> request) {
        requests_++;
        auto it = predicates.find(key(request->node));
        if (it != predicates.end()) {
          ExistNode::Response response;
          response.exist = it->second;
          service->send_response(*header, response);
        }
      });
    exist_function_ = create_service<ExistNode>(
      "problem_expert/exist_problem_function",
      [this](std::shared_ptr<rclcpp::Service<ExistNode>>,
      std::shared_ptr<rmw_request_id_t>, std::shared_ptr<ExistNode::Request>) {
        requests_++;
      });
    get_instances_ = create_service<GetProblemInstances>(
      "problem_expert/get_problem_instances",
      [this](std::shared_ptr<rclcpp::Service<GetProblemInstances>> service,
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<GetProblemInstances::Request>) {
        requests_++;
        if (instances.has_value()) {
          GetProblemInstances::Response response;
          response.success = true;
          response.instances = instances.value();
          service->send_response(*header, response);
        }
      });
  }

  static std::string key(const plansys2_msgs::msg::Node & node)
  {
    std::string ret = node.name;
    for (const auto & param : node.parameters) {
      ret += " " + param.name;
    }
    return ret;
  }

  std::map<std::string, bool> predicates;
  std::optional<std::vector<plansys2_msgs::msg::Param>> instances;
  std::atomic<int> requests_ {0};

private:
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr state_;
  rclcpp::Service<ExistNode>::SharedPtr exist_predicate_;
  rclcpp::Service<ExistNode>::SharedPtr exist_function_;
  rclcpp::Service<GetProblemInstances>::SharedPtr get_instances_;
};

plansys2_msgs::msg::Tree tree_of(const std::string & expression)
{
  plansys2_msgs::msg::Tree ret;
  // A bare predicate needs a logical parent to be read as one
  parser::pddl::fromString(ret, expression, false, plansys2_msgs::msg::Node::AND);
  return ret;
}

// First, so that no problem expert has ever been seen in the process
TEST(ClientNoServerTest, calls_without_server_wait_for_it)
{
  // No problem expert yet: the call waits for it, and goes on as soon as it is active
  auto client_node = rclcpp::Node::make_shared("client_node");
  plansys2::testing::NodeSpinner client_node_spinner(client_node);
  auto client = std::make_shared<plansys2::ProblemExpertClient>(client_node);

  std::atomic<bool> done {false};
  std::optional<bool> answer;
  std::thread caller([&]() {
      answer = client->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp1)"));
      done = true;
    });
  std::this_thread::sleep_for(3s);
  EXPECT_FALSE(done.load());

  auto server = std::make_shared<PartialProblemExpert>();
  server->predicates["robot_at r1 wp1"] = true;
  plansys2::testing::NodeSpinner server_spinner(server);
  EXPECT_TRUE(plansys2::testing::wait_for([&]() {return done.load();}, 10s));
  caller.join();
  EXPECT_EQ(answer, std::optional<bool>(true));
}

class ClientNoAnswerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_shared<PartialProblemExpert>();
    client_node_ = rclcpp::Node::make_shared("client_node");
    client_node_spinner_ = std::make_unique<plansys2::testing::NodeSpinner>(client_node_);
    client_ = std::make_shared<plansys2::ProblemExpertClient>(client_node_);
    exe_.add_node(server_);
    spinner_ = std::make_unique<plansys2::testing::ExecutorThread>(exe_);
  }

  void TearDown() override
  {
    spinner_.reset();
    exe_.remove_node(server_);
    client_.reset();
    client_node_.reset();
    server_.reset();
    plansys2::drain_ros(500ms);
  }

  std::shared_ptr<PartialProblemExpert> server_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<plansys2::testing::NodeSpinner> client_node_spinner_;
  std::shared_ptr<plansys2::ProblemExpertClient> client_;
  rclcpp::executors::MultiThreadedExecutor exe_;
  std::unique_ptr<plansys2::testing::ExecutorThread> spinner_;
};

TEST_F(ClientNoAnswerTest, try_methods_tell_no_answer_apart)
{
  server_->predicates["robot_at r1 wp1"] = true;
  server_->predicates["robot_at r1 wp2"] = false;

  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp1)")), true);
  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp2)")), false);
  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp3)")), std::nullopt);
  EXPECT_EQ(client_->tryExistFunction(plansys2::Function("(= (battery r1) 1)")), std::nullopt);
  EXPECT_FALSE(client_->tryGetInstances().has_value());

  // The old methods keep their answers: false or empty without an answer
  EXPECT_TRUE(client_->existPredicate(plansys2::Predicate("(robot_at r1 wp1)")));
  EXPECT_FALSE(client_->existPredicate(plansys2::Predicate("(robot_at r1 wp3)")));
  EXPECT_FALSE(client_->existFunction(plansys2::Function("(= (battery r1) 1)")));
  EXPECT_TRUE(client_->getInstances().empty());

  // An empty problem is an answer
  server_->instances = std::vector<plansys2_msgs::msg::Param>{};
  auto instances = client_->tryGetInstances();
  ASSERT_TRUE(instances.has_value());
  EXPECT_TRUE(instances->empty());
}

TEST_F(ClientNoAnswerTest, answers_after_a_request_without_answer)
{
  // A request left without answer does not get in the way of the next ones
  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp3)")), std::nullopt);
  server_->predicates["robot_at r1 wp3"] = true;
  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp3)")), true);
  server_->predicates["robot_at r1 wp3"] = false;
  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp3)")), false);
}

TEST_F(ClientNoAnswerTest, condition_without_answer_is_never_true)
{
  // wp1 and wp2 are answered, wp3 never
  server_->predicates["robot_at r1 wp1"] = true;
  server_->predicates["robot_at r1 wp2"] = false;

  // Expression, truth value expected. Without the answer for wp3, nothing that depends on it
  // holds, negated or not
  const std::vector<std::pair<std::string, bool>> cases = {
    {"(robot_at r1 wp3)", false},
    {"(not (robot_at r1 wp3))", false},
    {"(and (robot_at r1 wp1) (robot_at r1 wp3))", false},
    {"(and (robot_at r1 wp1) (not (robot_at r1 wp3)))", false},
    {"(not (and (robot_at r1 wp1) (robot_at r1 wp3)))", false},
    {"(not (or (robot_at r1 wp2) (robot_at r1 wp3)))", false},
    {"(or (robot_at r1 wp2) (not (robot_at r1 wp3)))", false},
    {"(imply (robot_at r1 wp3) (robot_at r1 wp2))", false},
    {"(imply (robot_at r1 wp1) (not (robot_at r1 wp3)))", false},
    // What holds anyway still holds
    {"(or (robot_at r1 wp1) (robot_at r1 wp3))", true},
    {"(or (not (robot_at r1 wp2)) (not (robot_at r1 wp3)))", true},
    {"(not (and (robot_at r1 wp2) (robot_at r1 wp3)))", true},
  };

  for (const auto & [expression, expected] : cases) {
    auto condition = tree_of(expression);
    EXPECT_EQ(plansys2::check(condition, client_), expected) << expression;

    std::vector<plansys2::Predicate> predicates;
    std::vector<plansys2::Function> functions;
    auto [success, value, number] =
      plansys2::evaluate(condition, client_, predicates, functions, false, false, 0);
    (void)number;
    EXPECT_FALSE(success) << expression;
    EXPECT_EQ(value, expected) << expression;
  }
}

TEST_F(ClientNoAnswerTest, quantifier_without_instances_is_never_true)
{
  server_->predicates["robot_at r1 wp1"] = true;

  // Without the objects an empty forall would be true and a negated exists too
  for (const auto & expression : {
    "(forall (?r - robot) (robot_at ?r wp1))",
    "(exists (?r - robot) (robot_at ?r wp1))",
    "(not (exists (?r - robot) (robot_at ?r wp1)))",
    "(not (forall (?r - robot) (robot_at ?r wp1)))"})
  {
    auto condition = tree_of(expression);
    EXPECT_FALSE(plansys2::check(condition, client_)) << expression;
  }

  // With the objects they are evaluated as usual
  plansys2_msgs::msg::Param r1;
  r1.name = "r1";
  r1.type = "robot";
  server_->instances = std::vector<plansys2_msgs::msg::Param>{r1};
  EXPECT_TRUE(plansys2::check(tree_of("(forall (?r - robot) (robot_at ?r wp1))"), client_));
  EXPECT_TRUE(plansys2::check(tree_of("(exists (?r - robot) (robot_at ?r wp1))"), client_));
  EXPECT_FALSE(
    plansys2::check(tree_of("(not (exists (?r - robot) (robot_at ?r wp1)))"), client_));
}

TEST_F(ClientNoAnswerTest, a_request_without_answer_is_sent_three_times)
{
  // One second each, with a new client for the second and the third (a lost answer)
  auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(client_->tryExistPredicate(plansys2::Predicate("(robot_at r1 wp3)")), std::nullopt);
  auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GE(elapsed, 2900ms);
  EXPECT_LT(elapsed, 6s);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Before the process exits: Zenoh aborts if ROS is shut down by atexit
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
