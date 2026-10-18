(define (domain energy_aware)
  (:requirements :strips :typing :numeric-fluents)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?rm - room)
    (lift ?from ?to - room)
    (ramp ?from ?to - room)
    (cleaned ?rm - room)
  )
  (:functions
    (energy-used)
  )
  (:action take_lift
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (lift ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
      (increase (energy-used) 12)
    )
  )
  (:action drive_ramp
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (ramp ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
      (increase (energy-used) 3)
    )
  )
  (:action clean
    :parameters (?r - robot ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
    )
    :effect (and
      (cleaned ?rm)
      (increase (energy-used) 2)
    )
  )
)
