(define (domain ready_delivery)
  (:requirements :strips :typing :derived-predicates)
  (:types
    robot room package
  )
  (:predicates
    (robot_at ?r - robot ?rm - room)
    (connected ?from ?to - room)
    (charged ?r - robot)
    (calibrated ?r - robot)
    (charger_in ?rm - room)
    (calibration_target_in ?rm - room)
    (package_at ?p - package ?rm - room)
    (carrying ?r - robot ?p - package)
    (ready ?r - robot)
  )
  (:derived (ready ?r - robot)
    (and
      (charged ?r)
      (calibrated ?r)
    )
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
    )
  )
  (:action charge
    :parameters (?r - robot ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (charger_in ?rm)
    )
    :effect (and
      (charged ?r)
    )
  )
  (:action calibrate
    :parameters (?r - robot ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (calibration_target_in ?rm)
    )
    :effect (and
      (calibrated ?r)
    )
  )
  (:action pick
    :parameters (?r - robot ?p - package ?rm - room)
    :precondition (and
      (ready ?r)
      (robot_at ?r ?rm)
      (package_at ?p ?rm)
    )
    :effect (and
      (not (package_at ?p ?rm))
      (carrying ?r ?p)
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
    )
  )
)
