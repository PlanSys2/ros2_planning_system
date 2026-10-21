(define (domain battery_patrol)
  (:requirements :strips :typing :numeric-fluents)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?rm - room)
    (connected ?from ?to - room)
    (charger_in ?rm - room)
    (patrolled ?rm - room)
  )
  (:functions
    (battery ?r - robot)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
      (>= (battery ?r) 10)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
      (decrease (battery ?r) 10)
    )
  )
  (:action patrol
    :parameters (?r - robot ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (>= (battery ?r) 5)
    )
    :effect (and
      (patrolled ?rm)
      (decrease (battery ?r) 5)
    )
  )
  (:action recharge
    :parameters (?r - robot ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (charger_in ?rm)
      (<= (battery ?r) 50)
    )
    :effect (and
      (assign (battery ?r) 100)
    )
  )
)
