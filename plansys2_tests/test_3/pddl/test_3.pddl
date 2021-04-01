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
