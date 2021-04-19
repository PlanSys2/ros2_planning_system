# Test 3

Test with several flows performed by 3 robots, with different action performers per action.

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
(define (domain test_3)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
zone
piece
car
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_available ?r - robot)
(robot_at ?r - robot ?z - zone)
(piece_at ?p - piece ?z - zone)

(piece_is_wheel ?p - piece)
(piece_is_body_car ?p - piece)
(piece_is_steering_wheel ?p - piece)

(piece_not_used ?p - piece)

(is_assembly_zone ?z - zone)

(car_assembled ?c - car)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 2)
    :condition (and
        (at start(robot_available ?r))
        (at start(robot_at ?r ?z1)))
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
)

(:durative-action transport
    :parameters (?r - robot ?p - piece ?z1 ?z2 - zone)
    :duration ( = ?duration 3)
    :condition (and
        (at start(robot_available ?r))
        (at start(robot_at ?r ?z1))
        (at start(piece_at ?p ?z1))
    )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(piece_at ?p ?z1)))
        (at end(piece_at ?p ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
)

(:durative-action assemble
    :parameters (?r - robot ?z - zone ?p1 ?p2 ?p3 - piece ?c - car)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_available ?r))
        (at start(is_assembly_zone ?z))
        (at start(robot_at ?r ?z))

        (at start(piece_at ?p1 ?z))
        (at start(piece_at ?p2 ?z))
        (at start(piece_at ?p3 ?z))

        (at start(piece_not_used ?p1))
        (at start(piece_not_used ?p2))
        (at start(piece_not_used ?p3))

        (at start(piece_is_wheel ?p1))
        (at start(piece_is_body_car ?p2))
        (at start(piece_is_steering_wheel ?p3))
    )
    :effect (and
        (at start(not(piece_not_used ?p1)))
        (at start(not(piece_not_used ?p2)))
        (at start(not(piece_not_used ?p3)))
        (at end(car_assembled ?c))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))

    )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;


```

### Problem

```
( define ( problem problem_1 )
( :domain plansys2 )
( :objects
	robot1 robot2 robot3 - robot
	wheels_zone sterwheel_zone body_car_zone assembly_zone recharge_zone - zone
	wheel_1 wheel_2 wheel_3 body_car_1 body_car_2 body_car_3 sterwheel_1 sterwheel_2 sterwheel_3 - piece
	car1 car2 car3 - car
)
( :init
	( robot_at robot1 assembly_zone )
	( robot_at robot2 assembly_zone )
	( robot_at robot3 assembly_zone )
	( is_assembly_zone assembly_zone )
	( robot_available robot1 )
	( robot_available robot2 )
	( robot_available robot3 )
	( piece_at wheel_1 wheels_zone )
	( piece_at body_car_1 body_car_zone )
	( piece_at sterwheel_1 sterwheel_zone )
	( piece_is_wheel wheel_1 )
	( piece_is_body_car body_car_1 )
	( piece_is_steering_wheel sterwheel_1 )
	( piece_at wheel_2 wheels_zone )
	( piece_at body_car_2 body_car_zone )
	( piece_at sterwheel_2 sterwheel_zone )
	( piece_is_wheel wheel_2 )
	( piece_is_body_car body_car_2 )
	( piece_is_steering_wheel sterwheel_2 )
	( piece_at wheel_3 wheels_zone )
	( piece_at body_car_3 body_car_zone )
	( piece_at sterwheel_3 sterwheel_zone )
	( piece_is_wheel wheel_3 )
	( piece_is_body_car body_car_3 )
	( piece_is_steering_wheel sterwheel_3 )
	( piece_not_used wheel_1 )
	( piece_not_used wheel_2 )
	( piece_not_used wheel_3 )
	( piece_not_used body_car_1 )
	( piece_not_used body_car_2 )
	( piece_not_used body_car_3 )
	( piece_not_used sterwheel_1 )
	( piece_not_used sterwheel_2 )
	( piece_not_used sterwheel_3 )
)
( :goal
	( and
		( car_assembled car1 )
		( car_assembled car2 )
		( car_assembled car3 )
	)
)
)

```

### Plan

```
0.000: (move robot1 assembly_zone body_car_zone)  [2.000]
0.000: (move robot2 assembly_zone sterwheel_zone)  [2.000]
0.000: (move robot3 assembly_zone wheels_zone)  [2.000]
2.001: (transport robot1 body_car_1 body_car_zone assembly_zone)  [3.000]
2.001: (transport robot2 sterwheel_1 sterwheel_zone assembly_zone)  [3.000]
2.001: (transport robot3 wheel_1 wheels_zone assembly_zone)  [3.000]
5.002: (assemble robot1 assembly_zone wheel_1 body_car_1 sterwheel_1 car1)  [5.000]
5.002: (move robot2 assembly_zone body_car_zone)  [2.000]
5.002: (move robot3 assembly_zone sterwheel_zone)  [2.000]
7.003: (transport robot2 body_car_2 body_car_zone assembly_zone)  [3.000]
7.003: (transport robot3 sterwheel_2 sterwheel_zone assembly_zone)  [3.000]
10.003: (move robot1 assembly_zone wheels_zone)  [2.000]
10.004: (move robot3 assembly_zone body_car_zone)  [2.000]
12.004: (transport robot1 wheel_2 wheels_zone assembly_zone)  [3.000]
12.005: (transport robot3 body_car_3 body_car_zone assembly_zone)  [3.000]
15.005: (assemble robot2 assembly_zone wheel_2 body_car_2 sterwheel_2 car2)  [5.000]
15.005: (move robot1 assembly_zone sterwheel_zone)  [2.000]
15.006: (move robot3 assembly_zone wheels_zone)  [2.000]
17.006: (transport robot1 sterwheel_3 sterwheel_zone assembly_zone)  [3.000]
17.007: (transport robot3 wheel_3 wheels_zone assembly_zone)  [3.000]
20.008: (assemble robot1 assembly_zone wheel_3 body_car_3 sterwheel_3 car3)  [5.000]

```

