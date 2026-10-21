# Kairos Plan solver

Plan solver plugin that uses [Kairos](https://github.com/PlanSys2/kairos), a temporal and numeric
PDDL planner, as a library: no files are written and no process is started, so PlanSys2 examples
plan in a few milliseconds.

- Temporal (durative actions, timed initial literals), numeric and classical domains, derived
  predicates, conditional effects and quantifiers.
- Anytime: the first plan comes fast, then Kairos improves it (shorter makespan) until no better
  plan arrives for `no_improvement` seconds or the time runs out.
- Every plan is validated before it is returned.
- The parsed domains are cached (by hash and text), so replanning with the same domain does not
  parse it again; a changed domain is parsed again.
- `cancel()` stops the search at once and returns the best plan found so far, if any.
- `isDomainValid` checks the domain without planning, and logs errors with line and column.

## Usage

```yaml
planner:
  ros__parameters:
    plan_solver_plugins: ["KAIROS"]
    KAIROS:
      plugin: "plansys2/KairosPlanSolver"
```

It can run together with other solvers (e.g. `["KAIROS", "POPF"]`): the planner node returns the
plans of all of them.

## Parameters

All of them under the plugin name (e.g. `KAIROS.anytime`), read at each call:

| Parameter | Default | Meaning |
|---|---|---|
| `timeout` | `0.0` | Seconds per plan; `0` uses the planner node `plan_solver_timeout`. It never goes beyond that one: Kairos returns a little before the node deadline. |
| `anytime` | `true` | Keep improving the first plan. |
| `no_improvement` | `0.1` | With `anytime`, seconds without a better plan before returning the best one (`0`: until the timeout). Longer waits rarely give better plans in PlanSys2's problems and make every plan take at least that long. |
| `search` | `auto` | Search preset: `auto`, `ff`, `lama`, `eager-lama`, `novelty`, `lama-novelty`, `ehc`. |
| `engine` | `auto` | `auto`, `m1` (no durative actions), `m2` (compressed temporal), `m3` (snap actions, concurrency). |
| `memory_limit_mb` | `0` | Memory for the search; `0` is half of the available memory. |
| `threads` | `1` | Phases of the search at once. More threads find first plans sooner and better plans on temporal problems (180 instead of 172 of the 225 IPC temporal problems with 3), but use more cores; on a robot the computer is shared, so it stays at 1 unless set. |
| `log_level` | `warn` | Kairos messages forwarded to the ROS logger (`<node>.kairos`): `debug`, `info`, `warn`, `error`. |
| `dump_dir` | `""` | When set, Kairos writes its intermediate structures there (for debugging). |

Sequential plans (no durative actions) are returned with `time` = position (0, 1, 2...) and
`duration` = 0.
