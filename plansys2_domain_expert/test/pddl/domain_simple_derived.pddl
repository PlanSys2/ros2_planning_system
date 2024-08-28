(define (domain plansys2)
(:requirements :strips :typing :adl :derived-predicates)

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
(inferred-robot_at ?r - robot ?ro - room)
(person_at ?p - person ?ro - room)
(inferred-person_at ?p - person ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (teleportation_time ?from - teleporter_room ?to - room)
);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Derived predicates ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:derived (inferred-robot_at ?r - robot ?ro - room)
  (and
    (robot_at ?r ?ro)
  )
)
(:derived (inferred-person_at ?p - person ?ro - room)
  (and
    (person_at ?p ?ro)
  )
)

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :precondition (and
        (inferred-robot_at ?r1 ?r2)
        (robot_at ?r ?r1))
    :effect (and
        (not(robot_at ?r ?r1))
        (robot_at ?r ?r2)
    )
)


(:action talk_leia
    :parameters (?from ?p - person ?m - message)
    :precondition (and
        (robot_near_person leia ?p)
    )
    :effect (and
        (robot_talk leia ?m ?p)
    )
)


(:action talk_lema
    :parameters (?from ?p - person ?m - message)
    :precondition (and
        (robot_near_person lema ?p)
    )
    :effect (and
        (robot_talk lema ?m ?p)
    )
)


(:action approach
    :parameters (?r - robot ?ro - room ?p - person)
    :precondition (and
        (robot_at ?r ?ro)
        (person_at ?p ?ro)
    )
    :effect (and
        (robot_near_person ?r ?p)
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
