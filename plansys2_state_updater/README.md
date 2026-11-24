# plansys2_state_updater

Keeps the PlanSys2 problem in sync with the sensors of the robot, configured only by YAML.
Each rule reads one field of one topic, transforms it, and writes one fact of the problem with a
write policy.

The problem expert loads it as a plugin (a `plansys2::ProblemUpdater`), and it changes the problem
directly, inside it. Give the bringup the rules:

```bash
ros2 launch plansys2_bringup plansys2_bringup_launch_monolithic.py \
  model_file:=/path/domain.pddl state_updater_config:=/path/rules.yaml
```

which is the same as these parameters of the problem expert (in `params_file`):

```yaml
problem_expert:
  ros__parameters:
    updater_plugins: [state_updater]
    state_updater:
      plugin: plansys2_state_updater/RulesUpdater
      config_file: /path/rules.yaml
      dry_run: false            # logs what it would write, without touching the problem
      check_domain: true        # the rules against the domain, the problem and the plan
      resync_period: 2.0        # what the problem lost is written again (0: never)
      timer_period: 0.1         # debounce, min_period, stale and expire_after, at this period
      disabled_rules: []        # rules that write nothing, by name
      retry_after: 5.0          # a change the problem did not take, tried again after this
```

Its callbacks go in the callback group of the problem expert: never at the same time as its
services, so the problem is never read half changed. If the rules are not valid, the problem
expert does not configure, and says why.

### Plugins of your own

The problem expert loads any `plansys2::ProblemUpdater` of `updater_plugins`: a class of yours,
exported for `plansys2_problem_expert`, gets the node and the problem, subscribes to what it wants
and changes the problem as it wants. The rules updater is one of them. To keep its policies
(threshold, debounce, stale...) and only preprocess a value in a way of your own, a
`TransformPlugin` is enough: see [Transforms](#transforms).

Configuring, it checks the rules against the domain and the problem, and fails with every error
found (the problem expert does not configure): a predicate or function that is not in the domain or has another number of parameters; an
object of another type than its parameter; a predicate that the actions of the plan change (it
would fight the executor); a function that they change, without mode `correct` (unless it is in
the `measured_functions` of `problem_expert`); mode `correct` on a measured function. It warns of
an object that is not in the problem yet, and of mode `correct` without `min_change`.

The changes of each cycle are made at once in the problem (`ProblemExpert::applyChanges`): all or
none, with one notification of the change of the problem, so one replanning and one check of the
over all conditions. If they cannot be made at once (an object that is not in the problem), they
are made one by one: the one that fails is tried again 5 s later, the others are made.

Mode `correct` compares each reading with the value in the problem. Every `resync_period` (2 s)
it checks the facts and objects of its rules against the problem, and writes again what it lost
or someone else changed (the problem was cleared, replaced): each fact has one owner.

Each write is logged with its reason (`battery: 95 → 82, deviation 13 ≥ 2`) and published on
`state_updater/events` (its name in `updater_plugins`) as a `plansys2_msgs/StateUpdate`: that is the signal for a mission to replan on, not
`problem_expert/update_notify`, which the dynamics (`#t`) publish at 20 Hz. Each event also says
what the fact was before, as far as the rules know (`has_previous`, `previous_value`,
`previous_object`, `previous_holds`): what was written, what the problem had for mode `correct`
and for a resync, nothing for a first value. The state of each rule
and of each of its facts and objects is published as JSON on `state_updater/status` once a
second.

While it runs:

```bash
# Disable some rules (their facts stay as they are); [] enables them all again
ros2 param set /problem_expert state_updater.disabled_rules "['battery', 'leak']"
# Load the rules again, from config_file (it may be changed first): they are checked as on
# configure, and the old ones stay if the new ones are not valid
ros2 service call /state_updater/reload std_srvs/srv/Trigger
```

A reload keeps what each rule that is the same (its name and its YAML) knew: what it wrote,
what waits for its debounce, its objects, when its topic last spoke; the new and changed ones
start. The facts written by a rule that is no longer in the reloaded rules stay as they are.

When the domain changes (ChangeDomain), the rules are checked against it again: the ones that no
longer fit write nothing, with why in their status (`enabled: false`, `error`), until a domain
fits them again.

## Rules

```yaml
state_updater:
  rules:
    - name: battery                          # unique
      topic: /robot/battery
      type: sensor_msgs/msg/BatteryState     # optional: else, seen in the graph
      field: percentage                      # also pose.pose.position.z, cells[2].voltage;
                                             # without it, the whole message
      transform: {linear: {scale: 100.0}}    # optional
      target: {function: "(energy)"}         # or {predicate:}, {set:}, {enum:}, {instance:}
      policy: {mode: correct, min_change: 2.0}
      qos: {reliability: auto}               # optional: see QoS
```

See [config/example.yaml](config/example.yaml).

### Object fluents

A function whose value is an object, as the domain says (`(:functions (location) - waypoint)`),
takes a name: the field, or a `map`, a `range` or a plugin that gives one. It is written as
`(= (location) wp2)`; mode `correct` and `min_change` are for numbers.

```yaml
- name: location
  topic: /robot/waypoint                     # std_msgs/msg/String
  field: data
  target: {function: "(location)"}
```

### Many facts: templates, sets and enums

A **template** takes the parameters of the fact from the message: with a `[*]` path, each element
writes its own fact, with its own policy state (debounce, stale...). An element that does not
come in a message is left as it was.

```yaml
- name: sensors
  topic: /diagnostics                        # diagnostic_msgs/msg/DiagnosticArray
  field: "status[*]"                         # each entry
  transform: {compare: {field: level, op: "==", value: 0}}
  target: {predicate: "(sensor_ok {name})"}  # name of the entry
  policy: {debounce: 1.0, stale_after: 5.0, on_stale: "(sensor_stale {name})"}
```

A **set** makes the elements of a list the facts that hold: the new ones are added, the ones that
leave the list are removed. `{}` is the element itself; with a list of structs, `{from}` its field.

```yaml
- name: reachable
  topic: /reachable
  field: waypoints                           # ['wp1', 'wp3'], or 'legs[*]' for structs
  target: {set: "(reachable {})"}            # or "(blocked {from} {to})"
```

An **enum** keeps one of several facts true at a time, the one of the value: the one that held
goes before the new one holds, so that two never hold at once. With `values`, the first value
also removes the others, as the one the problem file had.

```yaml
- name: mode
  topic: /robot/mode
  field: data
  transform: {map: {values: {2: normal, 3: degraded}, default: lost}}
  target: {enum: "(mode_is {})", values: [normal, degraded, lost]}   # values: optional
```

The values must be valid PDDL names of objects of the problem: only instance rules create
objects. A template, a set, an enum or an instance rule owns the whole predicates it writes: no
other rule can write any `sensor_ok`.

### Objects: instances

An **instance** rule creates the objects the messages speak of, with some facts on them, and
removes them when they are not seen for `expire_after`, or at once when the message is the whole
list and they are not in it (`remove_when_missing`): each contact of the sonar, an object. A
fact that changes (the contact moves) goes, and the new one holds. An object to remove that the
goal or the rest of the plan in course uses is kept, and removed once they no longer use it:
removing it would take the parts of the goal on it, and leave the plan without it.

```yaml
- name: contacts
  topic: /sonar/contacts
  field: "contacts[*]"                         # each one an object; without [*], one a message
  target:
    instance: {name: "contact_{id}", type: contact}
    facts: ["(contact_at {self} {waypoint})"]  # {self}: the object
  policy:
    expire_after: 30.0                         # without it (or remove_when_missing), never removed
    remove_when_missing: false                 # true: the message has them all, one not in it goes
    max_instances: 50                          # no more objects than these: the rest, not created
```

The type may come from the element too, `type: "{class}"`: each object is created with the type
its message says, if it is a type of the domain (else it is not created, and the status of the
rule says why). An object keeps the type it was created with.

### Transforms

| Transform | Arguments | Gives |
|---|---|---|
| none | | the field as it is |
| `linear` | `scale` (1), `offset` (0), `round` (decimals) | number |
| `compare` | `op` (`<` `<=` `>` `>=` `==` `!=`), `value` | boolean |
| `map` | `values: {sensor value: fact value}`, `default` | what it maps to |
| `range` | `limits: [5, 30]`, `values: [shallow, mid, deep]` (one more than limits) | the value of its interval: below 5, from 5 to 30, from 30 up |
| `hysteresis` | `on_below` + `off_above`, or `on_above` + `off_below` | boolean |
| `plugin` | `function: "package/Class"`, `args: {...}` | what the plugin gives |

Every transform takes `field`, a field of the value: with `status[*]`, the `level` of each entry.

A **plugin** is a C++ class of a package, loaded with pluginlib when the rules are loaded (the
YAML is never evaluated): for what the other transforms cannot do. It gets the value of the field
(without the `field` of the rule, the whole message, as a map) and the `args` of the rule. What it
throws is an error of its rule, in its status. `plansys2_state_updater/MinOf` comes with the
package: the lowest of a list, of a field of its elements, not below a minimum.

```yaml
- name: cell
  topic: /robot/battery
  field: cells
  transform: {plugin: {function: plansys2_state_updater/MinOf,
                       args: {field: voltage, minimum: 3.0}}}
  target: {function: "(lowest_cell)"}
```

One of your own (see [src/plugins/min_of.cpp](src/plugins/min_of.cpp)):

```cpp
#include "plansys2_state_updater/transforms.hpp"
#include "pluginlib/class_list_macros.hpp"

class Depth : public plansys2_state_updater::TransformPlugin
{
public:
  plansys2_state_updater::Value apply(
    const plansys2_state_updater::Value & value, const plansys2_state_updater::Value & args)
  override
  {
    return value.as_number() * args.as_map().at("density").as_number();
  }
};

PLUGINLIB_EXPORT_CLASS(Depth, plansys2_state_updater::TransformPlugin)
```

exported in its `package.xml` with
`<plansys2_state_updater plugin="${prefix}/plugins.xml"/>`, and named `my_package/Depth` in its
`plugins.xml`.

### Policy

| Key | For | Meaning |
|---|---|---|
| `mode` | functions | `write` (default): compares with what it wrote. `correct`: compares with the value in the problem, which the plan also changes (`#t`), and writes only when the sensor is `min_change` away from it |
| `min_change` | functions | writes only changes at least this big (default: any change) |
| `debounce` | predicates | the new value must hold this many seconds before it is written |
| `min_period` | both | at most one write every this many seconds; the last value is written later |
| `stale_after` | both | seconds without messages to consider the topic stale |
| `on_stale` | all but sets | `keep` (default) the last value; a predicate such as `"(sensor_stale battery)"`, true while stale; `{value: 0.0}`, a value for the fact (a number, true or false, a value of an enum); or both, `{predicate: ..., value: ...}` |
| `sample_period` | all | a message every this many seconds at most is read; the others are not even read (a 100 Hz IMU, in the problem expert). Each rule on its own |
| `stamp` | all | the field with the time of the reading (`header.stamp`, or seconds): stale counts from it, not from when it came, and a reading older than `stale_after` (`expire_after`, for instances), or than one already taken, is not taken |

An instance rule takes `expire_after`, `remove_when_missing`, `max_instances`, `sample_period`
and `stamp`.

Each fact has a single owner: two rules cannot write the same fact.

A function with continuous effects (`#t`) in the domain is either predicted by the model and
corrected by a rule in mode `correct`, or listed in `measured_functions` of `problem_expert` and
written by a rule in mode `write`. Mode `correct` on a measured function is rejected: there is no
model to correct.

### QoS

How the topic of a rule is subscribed. A reading lost under load is a fact that is not
updated, so the subscription is reliable whenever its publishers are:

| Key | Values | Default |
|---|---|---|
| `reliability` | `auto`, `reliable`, `best_effort` | `auto` |
| `durability` | `auto`, `volatile`, `transient_local` | `volatile` |
| `depth` | messages kept (keep last) | `5` |

`auto` takes the most that all the publishers of the topic offer: reliable if all of them are
reliable, best effort if one of them is not, as most sensor drivers (`SensorDataQoS`), since a
reliable subscription gets nothing from a best-effort publisher; transient local only if all of
them are. The topic is subscribed when it has a publisher, and again when the publishers change
what they offer: a best-effort one that comes, or goes. With `reliable` or `best_effort` the
subscription is the one given, and a publisher that does not match it is warned of once.

A topic is subscribed once for all its rules: the rules of a topic that give `qos` must give
the same, and the ones that do not give it take it.

```yaml
- name: map_ready
  topic: /map_ready                          # latched: the last message, also to late joiners
  type: std_msgs/msg/Bool
  field: data
  target: {predicate: "(map_ready)"}
  qos: {reliability: reliable, durability: transient_local, depth: 1}
```

## A mission that replans

The state updater changes the problem, and the executor cuts an action whose over all conditions
no longer hold. What to do then is the mission's: [examples/replanning_mission.py](examples/replanning_mission.py)
plans for the goal of the problem, runs the plan, plans again when it fails, and when no plan
reaches the goal, waits for the world to change (`state_updater/events`) before asking the
planner again.

```bash
ros2 run plansys2_state_updater replanning_mission.py
```

## How it is made

A core without ROS nodes (`plansys2_state_updater_core`): it takes messages as values (maps,
lists, numbers, names) and the time, and gives the changes to write. Its modules: `value` (any
message, or YAML, as a value), `paths` (fields of messages, facts), `transforms`, `rules` (facts,
templates, sets, enums), `instances`, `domain_check` and `engine`. The plugin
(`plansys2_state_updater`) puts ROS around it: `message_reader` reads a message of any type, known
only by its name, by introspection; `problem_writer` makes the changes in the problem and reads
it; `domain_reader` reads what the checks need; `rules_updater` is the `ProblemUpdater` that
subscribes, ticks and publishes.

Tests: the core, the message reader and the plugin against a domain and a problem expert in the
same process, with gtest (`test/unit`); the plugin in the problem expert of a running PlanSys2,
its bringups and a mission, with pytest (`test/test_state_updater_*.py`).

## Not yet

A rule reads one topic: a fact made of two sensors needs a plugin of your own, or a node that
fuses them and publishes the result.
