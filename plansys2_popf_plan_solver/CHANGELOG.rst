^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_popf_plan_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2025-06-22)
------------------

3.0.0 (2025-06-06)
------------------
* Modify export target. Rename metapackage
* Revamp CMake and clean unused headers
* Update install
* Move pluginlib
* Update CMakeLists to manage dependencies more efficiently in various packages.
* Update CI adding BT.CPP as source dependecy
* Fix deprecation of ament_target_dependencies
* Closes `#352 <https://github.com/PlanSys2/ros2_planning_system/issues/352>`_
* Fix POPF plugin issue when retrieving a successful plan (`#352 <https://github.com/PlanSys2/ros2_planning_system/issues/352>`_) and add a message in the terminal for plans with no actions (but not empty).
* Multiple plans and replanning stability
* Planner Plugins can be cancelled. Plan timeout effective.
* Improve stability in Executor
* Support for existential preconditions
* adjust prints and utils
* Add parameter for timeout of plan solver
* Integrated feedback, and fixes to have tests to pass
* Fix CI after BT-CPP v4
* Added support for configuring the planner timeout
* Remove cmake warning
* Do not segfault with filesystem errors in POPF, and allow ~ for home directory
* Add unit tests
* Lint
* Do not segfault with filesystem errors in POPF, and allow ~ for home directory
* Add option to use planner node to validate domain in domain expert
* Add option to use planner node to validate domain in domain expert
* Add ability to specify output folder in POPF planner
* Reduce copypasta in domain expert
* Linting fixes
* Configure POPF solver in domain expert
* Add ability to specify output folder in POPF planner
* Merge remote-tracking branch 'upstream/master' into return-stn
* Update Changelog
* Remove reference to SharedPtr
* Fix is_valid_domain by parsing out file
* bt-builder-plugins: Creating BT builder plugin interface. Moving current BT builder to plugin named SimpleBTBuilder. Adding new and improved STN-based BT builder plugin named STNBTBuilder.
* Fix problem in is_valid_domain function
* add system status check
* add status check into popf_plan_solver
* Merge remote-tracking branch 'upstream/master'
* add semicolom
* add system status check
* Contributors: Alberto Tudela, Andrianov Roman, Francisco Martín Rico, Gustavo, Jake Keller, Josh Zapf, Marco Roveri, Robodrome, Sebastian Castro, Splinter1984, aquintan4


2.0.9 (2022-07-10)
------------------
* Humble Upgrade
* Update popf checker interface
* Contributors: Francisco Martín Rico

2.0.8 (2022-05-04)
------------------

2.0.7 (2022-05-04)
------------------

2.0.6 (2022-05-03)
------------------

2.0.5 (2022-05-03)
------------------
* Fix ROS2 Buildfarm error due to Threads
* Contributors: Francisco Martín Rico

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
* Merge galactic-devel
* Contributors: Francisco Martín Rico

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* pddl-tree-messages: Merging upstream master and resolving conflicts.
* Added ability to provide command-line arguments to popf via ROS parameter.
* Update version
* Contributors: Francisco Martín Rico, Greg Kogut, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Create tmp directories
* Add more verbose output to pddl errors
* Contributors: Fabrice Larribe, Francisco Martín Rico
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
* Contributors: Fabrice Larribe, Francisco Martin Rico, f269858

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
