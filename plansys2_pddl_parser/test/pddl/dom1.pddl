(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)
(:types
    robot
    room
)

(:predicates
    (robot_at ?r - robot ?ro - room)
    (connected ?ro1 ?ro2 - room)
    (battery_full ?r - robot)
    (charging_point_at ?ro - room)
)

(:constants
 rob1 rob2 - robot
 )

(:functions
    (battery_level ?r - robot)
)

(:durative-action action_test1
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
                (at start (and
                           (robot_at ?r ?r1)
                           (robot_at rob2 ?r1)))
                (at start(charging_point_at ?r2))
                (over all (and
                   (= ?r1 ?r2)
                   (= ?r rob1)
                   (= ?r2 ?r2)
                   (= 10 (battery_level ?r) )
                   (= (battery_level rob2) 20 )
                   (>= (battery_level rob1) 20 )
                   ))
                )
    :effect (and
        (at end (decrease (battery_level ?r) (* 1 0.5)))
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action action_test2
    :parameters (?r - robot ?ro - room)
    :duration ( = ?duration 20)
    :condition (and
        (at start(robot_at ?r ?ro))
        (at start(charging_point_at ?ro))
        (over all (and (> (battery_level ?r) 1) (< (battery_level ?r) 200)) )
    )
    :effect (and
         (at end (increase (battery_level ?r) (* ?duration 5.0)))
         (at end(battery_full ?r))
    )
)

(:durative-action action_test3
    :parameters (?r - robot)
    :duration ( = ?duration 20)
    :condition (and
        (at start
            (forall
                (?ro - room)
                (imply
                    (robot_at ?r ?ro)
                    (charging_point_at ?ro)
                )
            )
        )
        (over all (and (> (battery_level ?r) 1) (< (battery_level ?r) 200)) )
    )
    :effect (and
         (at end (increase (battery_level ?r) (* ?duration 5.0)))
         (at end (battery_full ?r))
    )
)

)