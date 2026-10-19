(define (domain timed_patrol)
  (:requirements :strips :typing :durative-actions :constraints)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?a - room ?b - room)
    (visited ?l - room)
    (lights_on ?l - room)
  )
  (:durative-action move
    :parameters (?r - robot ?from - room ?to - room)
    :duration (= ?duration 2)
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (visited ?to))
    )
  )
  (:durative-action switch_on
    :parameters (?r - robot ?l - room)
    :duration (= ?duration 1)
    :condition (and
      (over all (robot_at ?r ?l))
    )
    :effect (and
      (at end (lights_on ?l))
    )
  )
)
