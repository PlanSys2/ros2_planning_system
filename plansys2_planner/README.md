# Planner

The Planner module is responsible for creating plans. It delegates them to plan solver plugins. By default it uses [Kairos](https://github.com/PlanSys2/kairos), a temporal and numeric PDDL planner that runs as a library inside the node ([`plansys2_kairos_plan_solver`](../plansys2_kairos_plan_solver)). [POPF](https://github.com/fmrico/popf) is still available as a plugin ([`plansys2_popf_plan_solver`](../plansys2_popf_plan_solver)).

The main class is [`plansys2::PlannerNode`](include/include/plansys2_planner/PlannerNode.hpp), which is instantiated from [`planner_node.cpp`](src/planner_node.cpp). `plansys2::PlannerNode` is a also `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

Plan solvers are specified in the `plan_solver_plugins` parameter. In case of more than one specified, the first one will be used. If this parameter is not specified, Kairos will be used by default. To use POPF instead:

```yaml
planner:
  ros__parameters:
    plan_solver_plugins: ["POPF"]
    POPF:
      plugin: "plansys2/POPFPlanSolver"
```

## Services

- `/planner/get_plan` [[`plansys2_msgs::srv::GetPlan`](../plansys2_msgs/srv/GetPlan.srv)]
