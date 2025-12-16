(define (domain simple-door)
(:requirements :strips :typing :adl :fluents :durative-actions :existential-preconditions)

(:predicates
    (door_open)
    (not_door_open)
)

(:durative-action open_door
    :parameters ()
    :duration ( = ?duration 1)
    :condition (and
        (at start (not_door_open))
    )
    :effect (and
        (at start (door_open))
        (at start (not (not_door_open)))
    )
)

(:durative-action close_door
    :parameters ()
    :duration ( = ?duration 1)
    :condition (and
        (at start (door_open))
    )
    :effect (and
         (at start (not_door_open))
         (at start (not (door_open)))
    )
)
)
