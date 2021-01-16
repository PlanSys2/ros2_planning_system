# Planner

The Planner module is responsible for creating plans. By default, it uses [popf](https://github.com/fmrico/popf), that is a PDDL solver.

This module is very simple, as its only task is calling the popf binary and parsing the result.

The main class is [`plansys2::PlannerNode`](include/include/plansys2_planner/PlannerNode.hpp), which is instantiated from [`planner_node.cpp`](src/planner_node.cpp). `plansys2::PlannerNode` is a also `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

Plan solvers are specified in the `plan_solver_plugins` parameter. In case of more than one specified, the first one will be used. If this parameter is not specified, POPF will be used by default.

## Services

- `/planner/get_plan` [[`plansys2_msgs::srv::GetPlan`](../plansys2_msgs/srv/GetPlan.srv)]
