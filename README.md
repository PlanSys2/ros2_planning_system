![PlanSys2 Logo](/plansys2_docs/plansys2_logo.png)

[![Build Status](https://travis-ci.com/IntelligentRoboticsLabs/ros2_planning_system.svg?branch=master)](https://travis-ci.com/IntelligentRoboticsLabs/ros2_planning_system)


ROS2 Planning System (**plansys2** in short) is a project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system. It is implemented in ROS2, applying the latest concepts developed in this currently de-facto standard in Robotics.

This project is the result of several years of experience in the development of robotic behaviors using [ROSPlan](https://github.com/KCL-Planning/ROSPlan). ROSPlan has greatly inspired this project. In addition to the migration to ROS2, we contribute to key aspects: ease of use, efficiency, and new tools, such as our terminal.

We hope that this software helps to include planning in more Robotics projects, offering simple and powerful software to generate intelligent behaviors for robots.

We want to invite you to contribute to this Open Source project !!

# Requirements and compilation

This project was initially developed for ROS2 Eloquent. In addition to official packages, plansys2 requires popf, a PDDL plan solver, developed by Marc Hanheide, to which we have contributed to its migration to a ROS2 package.

Before compiling, include popf in your workspace:

```
plansys2_ws/src$ git clone https://github.com/LCAS/popf.git
```

Note: The previous URL may change, since Marc has generously decided to transfer ownership of this project, with the aim of revitalizing its development by including it in plansys2

Next, only compile:

```
plansys2_ws/src$ colcon build --symlink-install
```

<img src="/plansys2_docs/plansys2_logo_v2.png" alt="drawing" width="200" align="right"/>

