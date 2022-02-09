#ifndef BEHAVIOR_TREE__FAILURE_NODES_HPP_
#define BEHAVIOR_TREE__FAILURE_NODES_HPP_

#include <string>
#include <memory>

#include "test_msgs/action/fibonacci.hpp"

#include "plansys2_bt_actions/BTActionNode.hpp"

namespace plansys2_bt_tests
{

class OnTickFail : public plansys2::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  using Fibonacci = test_msgs::action::Fibonacci;
  explicit OnTickFail(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : plansys2::BtActionNode<Fibonacci>(xml_tag_name,
                                        action_name,
                                        conf)
  {
  }

  void on_tick() override
  {
    return_failure_ = true;
  }
};


class OnFeedbackFail : public plansys2::BtActionNode<test_msgs::action::Fibonacci>
{
public:
  using Fibonacci = test_msgs::action::Fibonacci;
  explicit OnFeedbackFail(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : plansys2::BtActionNode<Fibonacci>(xml_tag_name,
                                        action_name,
                                        conf)
  {
  }

  void on_feedback(const std::shared_ptr<const Fibonacci::Feedback>) override
  {
    return_failure_ = true;
  }
};

} // namespace plansys2_bt_tests




#endif  // #ifndef BEHAVIOR_TREE__FAILURE_NODES_HPP_

