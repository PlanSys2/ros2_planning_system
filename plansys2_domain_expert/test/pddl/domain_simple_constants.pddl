(define (domain plansys2)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
person  - object 
message - object
; This bracket inside a comment, is here for testing purpose :-)
robot   - object
room    - object
teleporter_room - room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

(:constants
leia - robot
lema - robot
jack john - person
)
;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_talk ?r - robot ?m - message ?p - person)
(robot_near_person ?r - robot ?p - person)
(robot_at ?r - robot ?ro - room)
(person_at ?p - person ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (teleportation_time ?from - teleporter_room ?to - room)
);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1)))
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)


(:durative-action talk_leia
    :parameters (?from ?p - person ?m - message)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_near_person leia ?p))
    )
    :effect (and
        (at end(robot_talk leia ?m ?p))
    )
)


(:durative-action talk_lema
    :parameters (?from ?p - person ?m - message)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_near_person lema ?p))
    )
    :effect (and
        (at end(robot_talk lema ?m ?p))
    )
)


(:durative-action approach
    :parameters (?r - robot ?ro - room ?p - person)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_at ?r ?ro))
        (over all(person_at ?p ?ro))
    )
    :effect (and
        (at end(robot_near_person ?r ?p))
    )
)


(:action move_person_john
    :parameters (?r1 ?r2 - room)
    :precondition (and 
        (person_at john ?r1)
    )
    :effect (and
        (person_at john ?r2)
        (not(person_at john ?r1))
    )
)


(:action move_person_jack
    :parameters (?r1 ?r2 - room)
    :precondition (and 
        (person_at jack ?r1)
    )
    :effect (and
        (person_at jack ?r2)
        (not(person_at jack ?r1))
    )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
