^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2026-10-21)
------------------
* One version for Humble, Jazzy, Kilted and Lyrical (`#509 <https://github.com/PlanSys2/ros2_planning_system/pull/509>`_)
* Robust PlanSys2: one node per component, clients that wait, retry and see a lost plan (`#507 <https://github.com/PlanSys2/ros2_planning_system/pull/507>`_)
* Execute everything Kairos plans: conditional effects, durations, timed literals, object fluents, recursive derived predicates, PDDL+ and constraints (`#504 <https://github.com/PlanSys2/ros2_planning_system/pull/504>`_)
* Fixes in the executor, the build, the tests and the CI (`#502 <https://github.com/PlanSys2/ros2_planning_system/pull/502>`_)
* Fixes in the Python support, the terminal, and the rqt and TUI tools (`#500 <https://github.com/PlanSys2/ros2_planning_system/pull/500>`_)
* Build plansys2_tools against Qt6 when qt_gui_cpp uses it (`#420 <https://github.com/PlanSys2/ros2_planning_system/pull/420>`_)
* Contributors: Francisco Martín Rico

3.1.0 (2026-07-28)
------------------
* Adaptation to the new Qt version
* Deprecate BTAction with rate. Fix all warnings
* Fix RQT plugins
* Set C++23 as default
* tools: conditionally include .hpp (vs .h)
  Headers have been renamed in Rolling & Kilted.
* Contributors: Eshan Savla, Francisco Martín Rico, Gustavo, Gustavo Rezende, gavanderhoorn

3.0.0 (2025-06-06)
------------------
* Revamp CMake and clean unused headers
* Move pluginlib
* Update CMakeLists to manage dependencies more efficiently in various packages.
* Update CI adding BT.CPP as source dependecy
* Fix deprecation of ament_target_dependencies
* Multiple plans and replanning stability
* Linting and fixing tests
* Add stability to RQT Plan plugin
* Remve specification for C++17
* Fix CI after BT-CPP v4
* Contributors: Alberto Tudela, Francisco Martín Rico, Josh Zapf, Marco Roveri, Splinter1984


2.0.9 (2022-07-10)
------------------
* Humble Upgrade
* Contributors: Francisco Martín Rico

2.0.8 (2022-05-04)
------------------
* Add missing dependency
* Contributors: Francisco Martín Rico

2.0.7 (2022-05-04)
------------------

2.0.6 (2022-05-03)
------------------

2.0.5 (2022-05-03)
------------------
* Fix package dep
* Contributors: Francisco Martín Rico, Jake Keller, Marco Roveri

2.0.4 (2022-05-03)
------------------
* Fix version
* Fix ROS2 Buildfarm error due to Threads
* Adding missing build dependencies
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Francisco Martín Rico, Jake Keller, Marco Roveri

2.0.3 (2022-04-03)
------------------
* Qt5 dependencies
* Contributors: Francisco Martín Rico

2.0.2 (2022-04-03)
------------------
* Fixed compilation of main (working on galactic) to compile also on foxy
* RQT Plan viewer
* Logger tool - performers and plan
* Logger tool - knowledge, info and action hub
* Contributors: Francisco Martín Rico, Jake Keller, Marco Roveri

2.0.1 (2022-02-03)
------------------

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------

1.0.7 (2021-01-04)
------------------

1.0.6 (2020-12-29)
------------------

1.0.5 (2020-12-28)
------------------

1.0.4 (2020-12-24)
------------------

1.0.3 (2020-12-23 19:46)
------------------------

1.0.2 (2020-12-23 11:12)
------------------------

1.0.1 (2020-07-19)
------------------

0.0.7 (2020-03-26)
------------------

0.0.6 (2020-03-23)
------------------

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09 07:55)
------------------------

0.0.3 (2020-01-09 07:11)
------------------------

0.0.2 (2020-01-08)
------------------
