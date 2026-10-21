(define (domain inspection)
  (:requirements :strips :typing :universal-preconditions)
  (:types
    robot room
  )
  (:predicates
    (robot_at ?r - robot ?rm - room)
    (connected ?from ?to - room)
    (inspected ?rm - room)
    (report_sent ?r - robot)
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
  (:action inspect
    :parameters (?r - robot ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
    )
    :effect (and
      (inspected ?rm)
    )
  )
  (:action send_report
    :parameters (?r - robot)
    :precondition (and
      (forall (?rm - room) (inspected ?rm))
    )
    :effect (and
      (report_sent ?r)
    )
  )
)
