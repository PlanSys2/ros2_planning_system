# Test 2

Short lineal plan with 6 actions and sub-types

## PlanSys2

- [x] Regular PlanSys2 actions
- [ ] Regular PlanSys2 actions with ROS2 action client
- [ ] PlanSys2 BT actions
- [ ] PlanSys2 BT actions with ROS2 action client

## PDDL

- [x] Types
- [x] Sub-Types
- [ ] Durative actions
  - [x] at start req
  - [x] over all req
  - [ ] at end req
  - [x] at start effect
  - [x] at end effect

### Domain

```
(define (domain test2)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
room zone corridor - place
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at_room ?r - robot ?ro - room)
(robot_at_zone ?r - robot ?z - zone)
(robot_at_corridor ?r - robot ?c - corridor)
(robot_at ?r - robot ?p - place)
(connected ?p1 ?p2 - place)
(battery_full ?r - robot)
(battery_low ?r - robot)
(charging_point_at ?z - zone)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?p1 ?p2 - place)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?p1 ?p2))
        (at start(robot_at ?r ?p1))
        (over all(battery_full ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?p1)))
        (at end(robot_at ?r ?p2))
    )
)

(:durative-action askcharge
    :parameters (?r - robot ?p1 - place ?p2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?p1))
        (at start(charging_point_at ?p2))
       )
    :effect (and
        (at start(not(robot_at ?r ?p1)))
        (at start(robot_at ?r ?p2))
    )
)

(:durative-action charge
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z))
        (at start(charging_point_at ?z))
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
	room_1 room_2 - room
  corridor_1 - corridor
  zone_1_1 zone_1_2 zone_recharge - zone 
)
( :init
	( connected room_1 corridor_1 )
	( connected corridor_1 room_1 )
	( connected room_2 corridor_1 )
	( connected corridor_1 room_2 )
  ( connected room_1 zone_1_1 )
	( connected zone_1_1 room_1 )
  ( connected room_1 zone_1_2 )
	( connected zone_1_2 room_1 )
  ( connected room_2 zone_recharge )
	( connected zone_recharge room_2 )
	( charging_point_at zone_recharge )
	( battery_low leia )
	( robot_at leia zone_1_1 )
)
( :goal
	( and
		( robot_at leia zone_1_2 )
	)
)
)
```

### Plan

```
0.000: (askcharge leia zone_1_1 zone_recharge)  [5.000]
0.001: (charge leia zone_recharge)  [5.000]
5.002: (move leia zone_recharge room_2)  [5.000]
10.003: (move leia room_2 corridor_1)  [5.000]
15.004: (move leia corridor_1 room_1)  [5.000]
20.005: (move leia room_1 zone_1_2)  [5.000]

```

