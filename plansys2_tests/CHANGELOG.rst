^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2025-06-22)
------------------

3.0.0 (2025-06-06)
------------------
* Revamp CMake and clean unused headers
* Update CMakeLists to manage dependencies more efficiently in various packages.
* Update CI adding BT.CPP as source dependecy
* Fix deprecation of ament_target_dependencies
* Multiple plans and replanning stability
* Replanning
* Adaptation of the rest of the packages
* Merge pull request `#332 <https://github.com/PlanSys2/ros2_planning_system/issues/332>`_ from PlanSys2/maintenance_rolling
  Maintenance rolling
* Change to EventsExecutor
* 🎨 linter for plansys2_pddl_parser
* Fix CI after BT-CPP v4
* Remove cmake warning
* Upgrade to Behaviortree.CPP v4
* Bump Behaviortree.CPP v3 to v4
* Contributors: Alberto Tudela, Francisco Martín Rico, Gustavo, Josh Zapf, Marco Roveri, robodrome


2.0.9 (2022-07-10)
------------------

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
* Fix compile issues for galactic
* Contributors: Francisco Martín Rico, bjnjo

1.0.10 (2021-07-03)
-------------------
* Remove node parameter in client constructors
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Replacing user access function calls with shorter versions where possible.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Merging master and resolving conflicts.
* Using custom behavior tree to enable action timeouts.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Tests with parallel execution
* Complete test2 description
* Added test for subtypes
* fix copy/paste
* Plansys2_tests package
* Contributors: Francisco Martín Rico, Josh Zapf
