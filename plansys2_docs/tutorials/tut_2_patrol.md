# Patrolling example

[This package](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/tree/master/plansys2_patrol_navigation_example) contains a more complex  example that uses ROS2 Navigation to make a robot patrol.

In one terminal run:

``` shell
ros2 launch patrol_navigation_example patrol_example_launch.py
```

This command launchs plansys2 and the nodes that implements the actions (move and patrol). The navigation param `autostart` is set to `true`, and the amcl is set to the starting position of the robot. This position is `wp_control`. We will use the next waypoints (x, y, yaw):

- wp_control (-2.0, -4.0, 0.0) - This is the starting point
- wp1 (0.0, -2.0, 0.0)
- wp2 (1.8, 0.0, 0.0)
- wp3 (0.0, 2.0, 0.0)
- wp4 (-0.5, 0.5, 0.0)

The setup from symbol to metric coordinates are made in the contructior of [MoveAction](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/move_action_node.cpp). They are stored in a std::map to get the metric coordinate of the commanded waypoint when the action is activated. The waypoint is an argument of the action.

The [MoveAction](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/move_action_node.cpp) action calls to the navigation 2 stack to make the robot move.

The [Patrol](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/patrol_action_node.cpp) action only makes the robot turn few seconds, by sending `geometry_msgs::msg::Twist` to `/cmd_vel`.

Next, we will run the node of the application, [patrolling controller node](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/patrolling_controller_node.cpp). This controls the phase of the behavior of the robot. It is implemented with a Finite State Machine (FSM). In each state, it sets a goal (`(and(patrolled wp1))`, for example), and calls to executor to generate a plan and execute it. The `init_knowledge()` method sets the connections among waypoints (all the navigations from a waypoint to another has to visit `wp_control`):

``` c++
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp_control)"));
```

To run this node, only type in another terminal:

``` shell
ros2 run patrol_navigation_example patrolling_controller_node
```

When the robot visit wp_4, it starts again the patrolling.

You have to see something like this:

[![Patrolling example](https://img.youtube.com/vi/fAEGySqefwo/0.jpg)](https://www.youtube.com/watch?v=fAEGySqefwo)
