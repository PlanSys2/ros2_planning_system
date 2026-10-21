(define (domain charging_station)
  (:requirements :strips :typing :numeric-fluents :time)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?a - room ?b - room)
    (has_charger ?l - room)
    (idle ?r - robot)
    (charging ?r - robot)
  )
  (:functions
    (battery ?r - robot)
  )
  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
      (idle ?r)
      (>= (battery ?r) 30)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
      (decrease (battery ?r) 30)
    )
  )
  (:action start_charge
    :parameters (?r - robot ?l - room)
    :precondition (and
      (robot_at ?r ?l)
      (has_charger ?l)
      (idle ?r)
    )
    :effect (and
      (not (idle ?r))
      (charging ?r)
    )
  )
  (:process recharge
    :parameters (?r - robot)
    :precondition (and
      (charging ?r)
    )
    :effect (and
      (increase (battery ?r) (* #t 10))
    )
  )
  (:event battery_full
    :parameters (?r - robot)
    :precondition (and
      (charging ?r)
      (>= (battery ?r) 100)
    )
    :effect (and
      (not (charging ?r))
      (idle ?r)
    )
  )
)
