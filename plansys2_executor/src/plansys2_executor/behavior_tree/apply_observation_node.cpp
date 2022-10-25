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

#include <string>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/apply_observation_node.hpp"

namespace plansys2 {

  ApplyObservation::ApplyObservation(
      const std::string &xml_tag_name,
      const BT::NodeConfiguration &conf)
      : ActionNodeBase(xml_tag_name, conf) {
    action_map_ =
        config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
            "action_map");

    problem_client_ =
        config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
            "problem_client");
  }

  BT::NodeStatus ApplyObservation::tick() {
    std::string value;
    std::string observe;
    getInput("observe", observe);
    getInput("value", value);

    problem_client_->removeConditional(Unknown("(unknown " + observe + ")"));
    auto observe_pred = parser::pddl::fromStringPredicate(observe);
    if (value == "true") {
      problem_client_->addPredicate(observe_pred);
    } else {
      // do nothing
    }

    return BT::NodeStatus::SUCCESS;

  }

}  // namespace plansys2
