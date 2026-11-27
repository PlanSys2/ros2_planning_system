# Klimax Plan solver

Plan solver plugin that uses [Klimax](https://github.com/PlanSys2/klimax), a planner of
hierarchical tasks (HTN) in HDDL, as a library. It is the planner of PlanSys2's HDDL mode
(see below); it does not take PDDL problems.

The plan it returns has its actions in `items`, as any other, and its decomposition in `tasks`
(`plansys2_msgs/PlanTask`): from the initial task network down to the actions, every task with
the method that decomposes it, the order the method gives its subtasks and the precondition of
the method with the values of its parameters. The `HTNBTBuilder` of the executor runs it so.

Plans with durative actions come with the times Klimax gives them (actions overlap where the
hierarchy and what they read and write allow); others, one action after another.

## Usage

```yaml
planner:
  ros__parameters:
    plan_solver_plugins: ["KLIMAX"]
    KLIMAX:
      plugin: "plansys2/KlimaxPlanSolver"
```

The bringup launch files set this by themselves in HDDL mode.

## Parameters

Under the plugin name (e.g. `KLIMAX.anytime`), read at each call:

| Parameter | Default | Meaning |
|---|---|---|
| `timeout` | `0.0` | Seconds per plan; `0` uses the planner node `plan_solver_timeout`. Klimax returns a little before the node deadline. |
| `anytime` | `false` | Go on after the first plan, looking for better ones (shorter, or cheaper by the metric of the problem) until the timeout or until one is proved the best. |

## HDDL mode

PlanSys2 plans and executes HDDL when both of these hold, and refuses to start when only one
does:

- the model files are `.hddl` (and the problem file, if any);
- the executor builds its trees with the `HTNBTBuilder`.

```
ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
  model_file:=house.hddl problem_file:=house_problem.hddl bt_builder_plugin:=HTNBTBuilder
```

In that mode:

- The domain expert keeps the tasks and methods of the domain as written, for the planner, and
  reads its actions as those of any PDDL domain: they are executed as always.
- The problem expert keeps the initial task network (`:htn`) of the last problem that had one,
  and writes it in the problem it gives the planner. Instances, predicates and goals are as
  always.
- The planner is Klimax.
- The executor runs the plan by its decomposition. Every task is a node of the tree, named after
  the task and its method, that fails, and fails those above it, when one of its subtasks
  fails. Before its subtasks run, the precondition of its method is checked in the state of
  then: if the world is no longer as when the plan was made, the task fails there, before any of
  its actions. The subtasks of a method run in the order the method gives them, at once where it
  does not order them.

Replanning: `executor/get_replanning_problem` gives, for a hierarchical plan, the problem of what
is left. Its task network is the frontier of the plan's tree: the tasks of which no action has
ended, whole; of those done in part, their subtasks; the actions still running, as tasks; each
with the orderings of the methods above it. The actions that go on are kept in the new plan.

## Building

Klimax is in `dependency_repos.repos`. It is built on Kairos' modules: Kairos installs them by
default (`KAIROS_INSTALL_CORE`).
