#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SEQUENTIAL_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SEQUENTIAL_BT_BUILDER_HPP_

#include "plansys2_executor/BTBuilder.hpp"

namespace plansys2
{
namespace bt_builder
{

class SequentialBTBuilder : public BTBuilder
{
public:
  SequentialBTBuilder();

  void initialize(
      const std::string & bt_action_1 = "", const std::string & bt_action_2 = "", int precision = 3);
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);
  plansys2::bt_builder::Graph::Ptr get_graph() {return nullptr;}
  bool propagate(plansys2::bt_builder::Graph::Ptr) {return true;}
  std::string get_dotgraph(
      std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
      bool enable_legend = false, bool enable_print_graph = false);

  std::string add_action_to_bt(
    const plansys2_msgs::msg::PlanItem & plan_item, 
    const std::vector<plansys2_msgs::msg::PlanItem> & previous_items);

protected:
  std::string bt_action_;
// plansys2::bt_builder::ActionGraph::Ptr get_graph(const plansys2_msgs::msg::Plan & current_plan);
};

}  // namespace bt_builder
}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::bt_builder::SequentialBTBuilder, plansys2::bt_builder::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SEQUENTIAL_BT_BUILDER_HPP_