(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(connected ?wp1 ?wp2 - waypoint)
(patrolled ?wp - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?wp1 ?wp2))
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

(:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(patrolled ?wp))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
