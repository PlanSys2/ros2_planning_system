// Copyright 2022 Intelligent Robotics Lab
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
#include <memory>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "FailureNodes.hpp"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys2_bt_tests::OnTickFail>(
        name, "move", config);
    };

  factory.registerBuilder<plansys2_bt_tests::OnTickFail>(
    "OnTickFail", builder);

  builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plansys2_bt_tests::OnFeedbackFail>(
        name, "move", config);
    };

  factory.registerBuilder<plansys2_bt_tests::OnFeedbackFail>(
    "OnFeedbackFail", builder);
}
