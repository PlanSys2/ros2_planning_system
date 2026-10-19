(define (domain negative_goals_cleaning)
  (:requirements :strips :typing :negative-preconditions)
  (:types
    robot room - object
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (dirty ?l - room)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action clean
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l) (dirty ?l))
    :effect (and (not (dirty ?l)))
  )
)
