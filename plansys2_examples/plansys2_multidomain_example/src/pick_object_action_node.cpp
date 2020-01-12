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

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PickObject : public plansys2::ActionExecutorClient
{
public:
  PickObject()
  : plansys2::ActionExecutorClient("pick_object")
  {
    getFeedback()->progress = 0.0;
  }

private:
  void actionStep()
  {
    if (getFeedback()->progress < 100.0) {
      getFeedback()->progress += 5.0;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Picking object ... [" << getFeedback()->progress << "%]  " << std::flush;
  }

  bool isFinished()
  {
    return getFeedback()->progress >= 100.0;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickObject>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
