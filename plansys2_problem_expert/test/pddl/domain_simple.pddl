(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
person - object
message - object
robot - object
room - object
room_with_teleporter - room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_talk ?r - robot ?m - message ?p - person)
(robot_near_person ?r - robot ?p - person)
(robot_at ?r - robot ?ro - room)
(person_at ?p - person ?ro - room)
(is_teleporter_enabled ?r - room_with_teleporter)
(is_teleporter_destination ?r - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (room_distance ?r1 - room ?r2 - room)
);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration (* (room_distance ?r1 ?r2) 0.5))
    :condition (and
        (at start(robot_at ?r ?r1)))
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:action teleport
    :parameters (?r - robot ?r1 - room_with_teleporter ?r2 - room)
    :precondition (and
            (robot_at ?r ?r1)
            (is_teleporter_enabled ?r1)
            (is_teleporter_destination ?r2)
            )
    :effect (and
        (not(robot_at ?r ?r1))
        (robot_at ?r ?r2)
    )
)

(:durative-action talk
    :parameters (?r - robot ?from ?p - person ?m - message)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_near_person ?r ?p))
    )
    :effect (and
        (at end(robot_talk ?r ?m ?p))
    )
)

(:durative-action approach
    :parameters (?r - robot ?ro - room ?p - person)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_at ?r ?ro))
        (over all(person_at ?p ?ro))
        (at end(person_at ?p ?ro))
    )
    :effect (and
        (at end(robot_near_person ?r ?p))
    )
)



);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
