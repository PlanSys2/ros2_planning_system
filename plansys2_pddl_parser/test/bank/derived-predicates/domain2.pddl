(define (domain obstacle_rooms)
  (:requirements :strips :typing :derived-predicates :negative-preconditions)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (obstacle ?l - room)
    (blocked ?l - room)
    (free ?l - room)
  )
  (:derived (free ?l - room)
    (not (blocked ?l))
  )
  (:derived (blocked ?l - room)
    (obstacle ?l)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
      (free ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )
  (:action clear
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
      (obstacle ?to)
    )
    :effect (and
      (not (obstacle ?to))
    )
  )
)
