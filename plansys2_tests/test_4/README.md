# Test 4

Short lineal plan with 1 action.

## PlanSys2

- [x] Regular PlanSys2 actions
- [ ] Regular PlanSys2 actions with ROS2 action client
- [ ] PlanSys2 BT actions
- [ ] PlanSys2 BT actions with ROS2 action client

## PDDL

- [ ] Types
- [x] Predicates
- [ ] Durative actions
  - [x] at start req
  - [ ] over all req
  - [ ] at end req
  - [ ] at start effect
  - [x] at end effect

### Domain

```
(define (domain simple-door)
(:requirements :strips :typing :adl :fluents :durative-actions :existential-preconditions)

(:predicates
    (door_open)
    (not_door_open)
)

(:durative-action open-door
    :parameters ()
    :duration ( = ?duration 1)
    :condition (and
        (at start (not_door_open))
    )
    :effect (and
        (at end (door_open))
    )
)

(:durative-action close-door
    :parameters ()
    :duration ( = ?duration 1)
    :condition (and
        (at start (door_open))
    )
    :effect (and
         (at end (not_door_open))
    )
)
)

```

### Problem

```
(define (problem open-the-door)
  (:domain simple-door)
  (:init
    (not_door_open)
  )
  (:goal
    (and (door_open))
  )
)
```

### Plan

```
0.000: (open-door)  [1.000]
```

