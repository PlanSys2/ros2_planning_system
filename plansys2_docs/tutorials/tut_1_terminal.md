# Working with Plansys2 and Terminal

Check out this [PDDL domain example](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/tree/master/plansys2_simple_example/pddl/simple_example.pddl). This is a small example of a tiny domain. It defines the types, predicates, and three actions for making a robot moving taking into account the battery level. It is a very basic example, but it is useful to illustrate this example.

Open a terminal and launch plansys2. We will use a launcher that includes the main planning system launcher, the specific action nodes for this example, and selects the domain:

``` shell
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py
```

or

``` shell
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py namespace:=my_namespace
```

if you want to launch it in `my_namespace` namespace.

Open other terminal and launch the plansys2 terminal:

``` shell
ros2 run plansys2_terminal plansys2_terminal
```

or, if you used a namespace

``` shell
ros2 run plansys2_terminal plansys2_terminal --ros-args -r __ns:=/my_namespace
```

The plansys2 terminal lets us operate directly against the planning system. It is a useful tool, useful to monitorize and developing your application. Usually, many of the next operations should be done inside your nodes. Plansys2 terminal is functional, but there is still too much to improve.

Inside the plansys2 terminal, you can check the domain:

``` plansys2_terminal
> get domain
```

You also can check the current instances in the plan, that now it's void:

``` plansys2_terminal
> get problem instances
```

To get the current predicates:

``` plansys2_terminal
> get problem predicates
```

Now, the problem is void. Let's add some content and recheck it after.

``` plansys2_terminal
> set instance leia robot
> set instance entrance room
> set instance kitchen room
> set instance bedroom room
> set instance dinning room
> set instance bathroom room
> set instance chargingroom room
> set predicate (connected entrance dinning)
> set predicate (connected dinning entrance)
> set predicate (connected dinning kitchen)
> set predicate (connected kitchen dinning)
> set predicate (connected dinning bedroom)
> set predicate (connected bedroom dinning)
> set predicate (connected bathroom bedroom)
> set predicate (connected bedroom bathroom)
> set predicate (connected chargingroom kitchen)
> set predicate (connected kitchen chargingroom)
> set predicate (charging_point_at chargingroom)
> set predicate (battery_low leia)
> set predicate (robot_at leia entrance)
```

Once added content to the problem, verify the content of the problem again:

``` plansys2_terminal
> get problem instances
...
> get problem predicates
```

Lets planify. Add a goal to be achieved:

``` plansys2_terminal
> set goal (and(robot_at leia bathroom))
```

And get the plan. This command will not execute the plan. Only will calculate it:

``` plansys2_terminal
> get plan
plan: 
0 (askcharge leia entrance chargingroom) 5
0.001 (charge leia chargingroom) 5
5.002 (move leia chargingroom kitchen) 5
10.003 (move leia kitchen dinning) 5
15.004 (move leia dinning bedroom) 5
20.005 (move leia bedroom bathroom) 5
```

To run the plan, type:

``` plansys2_terminal
run
```

You will see how the actions (calling to the nodes that implement the actions) are executed.

press Ctrl-D to exit.

The Plansys2 terminal has many functionalities: Adding/removing instances and predicates, asking for model details (predicates, actions, and types), run actions, getting plans, and running plans.
