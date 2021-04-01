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
