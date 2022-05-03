^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_pddl_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.6 (2022-05-03)
------------------
* Fix deps
* Contributors: Francisco Martín Rico

2.0.5 (2022-05-03)
------------------
* Fix ROS2 Buildfarm error due to Threads
* Contributors: Francisco Martín Rico, Jake Keller

2.0.4 (2022-05-03)
------------------
* Fix version
* Fix ROS2 Buildfarm error due to Threads
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Francisco Martín Rico, Jake Keller

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------
* Full supportr for (= _ _) PDDL constraints, added possibility to refer to constants, added initial test cases
* fix-getSubtrees: Removing empty line.
* fix-getSubtrees: Fixing bug in getSubtrees function in Utils.cpp.
* set_decimal_precision: Use std::setprecision to avoid incompatible scientific notation.
* [pddl_parser] ground constants also when no replace values
* [pddl_parser] account for pddl-constants while replacing ground params
* Contributors: Francisco Martín Rico, Jake Keller, Josh Zapf, Marco Roveri, Mostafa Gomaa

2.0.1 (2022-02-03)
------------------
* remove-invalid-goals: Removing invalid goals when instances are removed. Keep track of update time in problem expert.
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Josh Zapf

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------
* Solves error "error: 'bind' is not a member of 'std'"
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Adding a couple more utility functions to plansys2_pddl_parser.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Adding support for PDDL addition and subtraction expressions. Courtesy of @jjzapf
* Plansys2_tests package
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf, mfernandezcarmona@lincoln.ac.uk

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Adding unit tests for the Utils file. Fixing bug in plansys2_pddl_parser when getting predicate or function name that has no params it used to include the closing parenthesis in the name, which was incorrect. Making truth value false for expression arithmetic variants. fixing bug: not applying function modifier if trying to divide by zero.
* Add support for numeric conditions and effects.
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Josh Zapf

1.0.7 (2021-01-04)
------------------

1.0.6 (2020-12-29)
------------------

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

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Contributors: Francisco Martín Rico
0.0.8 (2020-07-18)
------------------

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Avoid inserting duplicate types
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
* Merge pull request `#16 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/16>`_ from IntelligentRoboticsLabs/pddl_parser_rename
  Rename pddl_parser
* Rename pddl_parser
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
