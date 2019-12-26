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

#include <memory>

#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING)
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp_control)"));
  }

  void step()
  {
    switch (state_) {
      case STARTING:


        if (true) {
          // Set the goal for next state, and execute plan
          problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

          if (executor_client_->executePlan()) {
            state_ = PATROL_WP1;
          }
        }
        break;
      case PATROL_WP1:
        {
          auto feedback = executor_client_->getFeedBack();

          std::cout << "[" << feedback.seq_action << "/" << feedback.total_actions << "]" <<
            "{" << feedback.current_action << "} [" << feedback.progress_current_action << "%]" <<
            std::endl;

          if (executor_client_->getResult().has_value()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp1)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp2))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP2;
              }
            } else {
              std::cout << "Finished with error: " <<
                executor_client_->getResult().value().error_info <<
                std::endl;
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      case PATROL_WP2:
        {
          auto feedback = executor_client_->getFeedBack();

          std::cout << "[" << feedback.seq_action << "/" << feedback.total_actions << "]" <<
            "{" << feedback.current_action << "} [" << feedback.progress_current_action << "%]" <<
            std::endl;

          if (executor_client_->getResult().has_value()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp2)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp3))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP3;
              }
            } else {
              std::cout << "Finished with error: " <<
                executor_client_->getResult().value().error_info <<
                std::endl;
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      case PATROL_WP3:
        {
          auto feedback = executor_client_->getFeedBack();

          std::cout << "[" << feedback.seq_action << "/" << feedback.total_actions << "]" <<
            "{" << feedback.current_action << "} [" << feedback.progress_current_action << "%]" <<
            std::endl;

          if (executor_client_->getResult().has_value()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp3)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp4))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP4;
              }
            } else {
              std::cout << "Finished with error: " <<
                executor_client_->getResult().value().error_info <<
                std::endl;
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      case PATROL_WP4:
        {
          auto feedback = executor_client_->getFeedBack();

          std::cout << "[" << feedback.seq_action << "/" << feedback.total_actions << "]" <<
            "{" << feedback.current_action << "} [" << feedback.progress_current_action << "%]" <<
            std::endl;

          if (executor_client_->getResult().has_value()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp4)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

              if (executor_client_->executePlan()) {
                // Loop to WP1
                state_ = PATROL_WP1;
              }
            } else {
              std::cout << "Finished with error: " <<
                executor_client_->getResult().value().error_info <<
                std::endl;
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      default:
        break;
    }
  }

private:
  typedef enum {STARTING, PATROL_WP1, PATROL_WP2, PATROL_WP3, PATROL_WP4} StateType;
  StateType state_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
