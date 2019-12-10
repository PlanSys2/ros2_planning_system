# Planner

The Planner module is responsible for creating plans. It uses [popf](https://github.com/fmrico/popf), that is is a PDDL solver.

This module is very simple, as its only task is calling the popf binary and parsing the result.

The main class is [`plansys2::PlannerNode`](include/include/plansys2_planner/PlannerNode.hpp), which is instantiated from [`planner_node.cpp`](src/planner_node.cpp). `plansys2::PlannerNode` is a also `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

The class responsible for creating plans is [`plansys2::Planner`](include/include/planner/Planner.hpp), which is independent of ROS2.

Before calling popf for calculating the plan, the domain is stored in `/tmp/domain.pddl`, and the problem in `/tmp/problem.pddl`. The plan generated is stored in `/tmp/plan` before parsing it.

The calling clients must provide the content of a domain and a problem. They can use a Domain Expert Client or a Problem Expert Client to get it:

## Services:

- `/planner/get_plan` [[`plansys2_msgs::srv::GetPlan`](../plansys2_msgs/srv/GetPlan.srv)]
