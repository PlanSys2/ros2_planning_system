# Terminal

The Plansys2 Terminal module is a tool to directly monitor and edit your application. See the tutorial [here](https://github.com/IntelligentRoboticsLabs/ros2_planning_system/blob/master/plansys2_docs/tutorials/tut_1_terminal.md). 

The node is created inside a Terminal object, which starts a looping run_console() method reading the terminal input and calling the corresponding services and actions.

To enable for using parameters specified in a YAML file for e.g. specifying the timeout for the planner and planner client, you can use the following command:
``ros2 run plansys2_terminal plansys2_terminal --ros-args --params-file `ros2 pkg prefix --share plansys2_bringup`/params/plansys2_params.yaml``

