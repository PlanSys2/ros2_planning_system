# Test 1

Short lineal plan with 6 actions.

## PlanSys2

- [x] Regular PlanSys2 actions
- [ ] Regular PlanSys2 actions with ROS2 action client
- [ ] PlanSys2 BT actions
- [ ] PlanSys2 BT actions with ROS2 action client

## PDDL

- [x] Types
- [ ] Durative actions
  - [x] at start req
  - [x] over all req
  - [ ] at end req
  - [x] at start effect
  - [x] at end effect

### Domain

```
(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?ro - room)
(connected ?ro1 ?ro2 - room)
(battery_full ?r - robot)
(battery_low ?r - robot)
(charging_point_at ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?r1 ?r2))
        (at start(robot_at ?r ?r1))
        (over all(battery_full ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action askcharge
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1))
        (at start(charging_point_at ?r2))
       )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at start(robot_at ?r ?r2))
    )
)

(:durative-action charge
    :parameters (?r - robot ?ro - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?ro))
        (at start(charging_point_at ?ro))
    )
    :effect (and
         (at end(not(battery_low ?r)))
         (at end(battery_full ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;

```

### Problem

```
( define ( problem problem_1 )
( :domain plansys2 )
( :objects
	leia - robot
	entrance kitchen bedroom dinning bathroom chargingroom - room
)
( :init
	( connected entrance dinning )
	( connected dinning entrance )
	( connected dinning kitchen )
	( connected kitchen dinning )
	( connected dinning bedroom )
	( connected bedroom dinning )
	( connected bathroom bedroom )
	( connected bedroom bathroom )
	( connected chargingroom kitchen )
	( connected kitchen chargingroom )
	( charging_point_at chargingroom )
	( battery_low leia )
	( robot_at leia entrance )
)
( :goal
	( and
		( robot_at leia bathroom )
	)
)
)
```

### Plan

```
0.000: (askcharge leia entrance chargingroom)  [5.000]
0.001: (charge leia chargingroom)  [5.000]
5.002: (move leia chargingroom kitchen)  [5.000]
10.003: (move leia kitchen dinning)  [5.000]
15.004: (move leia dinning bedroom)  [5.000]
20.005: (move leia bedroom bathroom)  [5.000]
```

