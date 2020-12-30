(define (domain plansys2)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
pickable_object
room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?ro - room)
(object_at_robot ?o - pickable_object ?r - robot)
(object_at_room ?o - pickable_object ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action pick_object
    :parameters (?r - robot ?ro - room ?o - pickable_object)
    :duration ( = ?duration 5)
    :condition (and
        (at start(object_at_room ?o ?ro))
        (at start(robot_at ?r ?ro))
        )
    :effect (and
        (at start(not(object_at_room ?o ?ro)))
        (at end(object_at_robot ?o ?r))
    )
)

(:durative-action place_object
    :parameters (?r - robot ?ro - room ?o - pickable_object)
    :duration ( = ?duration 5)
    :condition (and
        (at start(object_at_robot ?o ?r))
        (at start(robot_at ?r ?ro))
        )
    :effect (and
        (at start(not(object_at_robot ?o ?r)))
        (at end(object_at_room ?o ?ro))
    )
)
);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
