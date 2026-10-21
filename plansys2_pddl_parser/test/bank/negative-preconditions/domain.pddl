(define (domain negative_preconditions_patrol)
  (:requirements :strips :typing :negative-preconditions)
  (:types
    robot room - object
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (alarm ?l - room)
    (visited ?l - room)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to) (not (alarm ?to)))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action visit
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l) (not (visited ?l)))
    :effect (and (visited ?l))
  )
)
