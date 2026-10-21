(define (domain scheduled_charge)
  (:requirements :strips :typing :durative-actions :numeric-fluents :timed-initial-literals)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?a - room ?b - room)
  )
  (:functions
    (battery ?r - robot)
  )
  (:durative-action move
    :parameters (?r - robot ?from - room ?to - room)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (>= (battery ?r) 50))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at start (decrease (battery ?r) 50))
      (at end (robot_at ?r ?to))
    )
  )
)
