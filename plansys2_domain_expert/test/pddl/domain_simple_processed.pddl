(define (domain plansys2)
(:requirements :adl :durative-actions :fluents :strips :typing )

(:types
person  - object 
message - object
robot   - object
room    - object
teleporter_room - room
)

(:constants
)

(:predicates
(person_at ?p - person ?ro - room)
(robot_at ?r - robot ?ro - room)
(robot_near_person ?r - robot ?p - person)
(robot_talk ?r - robot ?m - message ?p - person)
)

(:functions
    (teleportation_time ?from - teleporter_room ?to - room)
)

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
    )
    :effect (and
        (at end(robot_near_person ?r ?p))
    )
)
(:action move_person
    :parameters (?p - person ?r1 ?r2 - room)
    :precondition (and 
        (person_at ?p ?r1)
    )
    :effect (and
        (person_at ?p ?r2)
        (not(person_at ?p ?r1))
    )
)
)
