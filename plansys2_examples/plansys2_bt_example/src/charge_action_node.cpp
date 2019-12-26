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
#include <string>

#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


class SayHello : public BT::ActionNodeBase
{
public:
  explicit SayHello(const std::string & name)
  : BT::ActionNodeBase(name, {})
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "Hello!! I would like to be charged. Please, plug me." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }
};

class ChargeRobot : public BT::ActionNodeBase
{
public:
  explicit ChargeRobot(const std::string & name)
  : BT::ActionNodeBase(name, {}), charge_level_(0.0)
  {
  }

  void reset()
  {
    charge_level_ = 0.0;
  }

  float get_level()
  {
    return charge_level_;
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    charge_level_ += 5.0;

    if (charge_level_ >= 100.0) {
      std::cout << "Robot completelly charged [" << charge_level_ << "]" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      std::cout << "Charging robot [" << charge_level_ << "]" << std::endl;
      return BT::NodeStatus::RUNNING;
    }
  }

  void halt() override
  {
  }

private:
  float charge_level_;
};

class SayGoodbye : public BT::ActionNodeBase
{
public:
  explicit SayGoodbye(const std::string & name)
  : BT::ActionNodeBase(name, {})
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "Thanks :) I will remember you when the Robot Uprising." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
  }
};


class Charge : public plansys2::ActionExecutorClient
{
public:
  Charge()
  : plansys2::ActionExecutorClient("charge")
  {
    factory_.registerNodeType<SayHello>("SayHello");
    factory_.registerNodeType<ChargeRobot>("ChargeRobot");
    factory_.registerNodeType<SayGoodbye>("SayGoodbye");

    std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_example");
    std::string xml_file = pkgpath + "/behavior_trees/charge_action_tree.xml";

    tree_ = factory_.createTreeFromFile(xml_file);
  }

private:
  void onActivate()
  {
    for (auto & node : tree_.nodes) {
      if (auto action_charge_robot = dynamic_cast<ChargeRobot *>(node.get())) {
        charge_robot_node_ = action_charge_robot;
        action_charge_robot->reset();
      }
    }

    finished_ = false;
  }

  void actionStep()
  {
    std::cerr << "AskCharge::actionStep()" << std::endl;
    finished_ = tree_.root_node->executeTick() == BT::NodeStatus::SUCCESS;
    getFeedback()->progress = charge_robot_node_->get_level();
  }

  bool isFinished()
  {
    return finished_;
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  bool finished_;

  ChargeRobot * charge_robot_node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Charge>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
