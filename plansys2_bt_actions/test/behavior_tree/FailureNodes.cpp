#include "behaviortree_cpp_v3/bt_factory.h"

#include "FailureNodes.hpp"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<plansys2_bt_tests::OnTickFail>(
        name, "move", config);
    };

  factory.registerBuilder<plansys2_bt_tests::OnTickFail>(
    "OnTickFail", builder);

  builder =
    [](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<plansys2_bt_tests::OnFeedbackFail>(
        name, "move", config);
    };

  factory.registerBuilder<plansys2_bt_tests::OnFeedbackFail>(
    "OnFeedbackFail", builder);
}

