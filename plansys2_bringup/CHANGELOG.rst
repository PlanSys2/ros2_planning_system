^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2025-06-22)
------------------

3.0.0 (2025-06-06)
------------------
* Revamp CMake and clean unused headers
* Update CMakeLists to manage dependencies more efficiently in various packages.
* Update CI adding BT.CPP as source dependency
* Fix deprecation of ament_target_dependencies
* Multiple plans and replanning stability
* Option to execute in Real Time
* Remove unnecessary specification for C++17
* Change to EventsExecutor
* Add parameter for timeout of plan solver
* Integrated feedback, and fixes to have tests to pass
* Added support for configuring the planner timeout
* Use separate problem pddl for problem expert.
* Use separate problem pddl for problem expert.
* Simplifying bound checkin logic.
* Fix bt node
* Make explicit which BTCreator is being used
* New BT Builder and Plugin Interface
* bt-builder-plugins: Setting default BT builder plugin to SimpleBTBuilder.
* bt-builder-plugins: Creating BT builder plugin interface. Moving current BT builder to plugin named SimpleBTBuilder. Adding new and improved STN-based BT builder plugin named STNBTBuilder.
* add system status check
* revert if statement
* Merge remote-tracking branch 'upstream/master'
* Contributors: Alberto Tudela, Francisco Martín Rico, Josh Zapf, Marco Roveri, Splinter1984, adfea625

2.0.9 (2022-07-10)
------------------
* Expose lifecyclemanager timeout as a parameter, other misc fixes/features
* Change bringup node to return an error on failure
* Add timeout to startup_function
* Rename function 'startup_script'
* change 'startup_script'... function return to bool
* Contributors: Francisco Martín Rico, Jake Keller

2.0.8 (2022-05-04)
------------------

2.0.7 (2022-05-04)
------------------

2.0.6 (2022-05-03)
------------------

2.0.5 (2022-05-03)
------------------

2.0.4 (2022-05-03)
------------------
* Fix version
* Fix ROS2 Buildfarm error due to Threads
* Contributors: Francisco Martín Rico

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------

2.0.1 (2022-02-03)
------------------
* Update deprecated APIs in launchers and parameters
* Update launcher param names
* Merge galactic-devel
* Contributors: Francisco Martín Rico

2.0.0 (2021-07-04)
------------------
* Compile for ROS2 Galactic
* Contributors: Francisco Martín Rico, Jonatan Olofsson

1.0.10 (2021-07-03)
-------------------
* pddl-tree-messages: Merging master and resolving conflicts.
* Using custom behavior tree to enable action timeouts.
* pddl-tree-messages: Merging upstream master and resolving conflicts.
* Making TFD plugin type match popf.
* linting
* Configurable action BT
* Configurable BT Action
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Update API for FutureReturnCode
* Add support for numeric conditions and effects.
* Contributors: Fabrice Larribe, Francisco Martin Rico, Josh Zapf

1.0.7 (2021-01-04)
------------------

1.0.6 (2020-12-29)
------------------
* Disable boost functions
* Contributors: Francisco Martín Rico
1.0.5 (2020-12-28)
------------------
* Migration to c++17
* Contributors: Francisco Martín Rico

1.0.4 (2020-12-24)
------------------

1.0.3 (2020-12-23)
------------------

1.0.2 (2020-12-23)
------------------
* Plan solvers as plugins
* Contributors: Fabrice Larribe, Francisco Martin Rico, Stephen Balakirsky

1.0.1 (2020-07-19)
------------------

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Boost:optional
* Contributors: Francisco Martin Rico

0.0.8 (2020-07-18)
------------------
* Foxy initial version
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Make mandatory to specify model
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Run in separate namespaces. Monolothic node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09)
------------------

0.0.3 (2020-01-09)
------------------

0.0.2 (2020-01-08)
------------------
* Merge pull request `#8 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/8>`_ from IntelligentRoboticsLabs/patrol_example
  Patrol example
* Patrol example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Execute actions independiently. Example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Executor initial version
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Bringup and lifecycle manager
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
