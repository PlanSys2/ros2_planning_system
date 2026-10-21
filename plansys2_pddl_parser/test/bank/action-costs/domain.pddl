(define (domain costly_delivery)
  (:requirements :strips :typing :numeric-fluents :action-costs)
  (:types
    robot room package
  )
  (:predicates
    (robot_at ?r - robot ?rm - room)
    (connected ?from ?to - room)
    (package_at ?p - package ?rm - room)
    (carrying ?r - robot ?p - package)
  )
  (:functions
    (distance ?from ?to - room)
    (total-cost)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
      (increase (total-cost) (distance ?from ?to))
    )
  )
  (:action pick
    :parameters (?r - robot ?p - package ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (package_at ?p ?rm)
    )
    :effect (and
      (not (package_at ?p ?rm))
      (carrying ?r ?p)
      (increase (total-cost) 1)
    )
  )
  (:action drop
    :parameters (?r - robot ?p - package ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (carrying ?r ?p)
    )
    :effect (and
      (not (carrying ?r ?p))
      (package_at ?p ?rm)
      (increase (total-cost) 1)
    )
  )
)
