(define (domain route_dispatch)
  (:requirements :strips :typing :derived-predicates :existential-preconditions)
  (:types
    robot waypoint door
  )
  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (link ?d - door ?from ?to - waypoint)
    (door_at ?d - door ?w - waypoint)
    (corridor ?from ?to - waypoint)
    (reachable ?from ?to - waypoint)
  )
  (:derived (reachable ?from ?to - waypoint)
    (and
      (corridor ?from ?to)
    )
  )
  (:derived (reachable ?from ?to - waypoint)
    (exists (?mid - waypoint)
      (and
        (corridor ?from ?mid)
        (reachable ?mid ?to)
      )
    )
  )
  (:action open_door
    :parameters (?r - robot ?d - door ?w ?from ?to - waypoint)
    :precondition (and
      (robot_at ?r ?w)
      (door_at ?d ?w)
      (link ?d ?from ?to)
    )
    :effect (and
      (corridor ?from ?to)
    )
  )
  (:action navigate
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and
      (robot_at ?r ?from)
      (reachable ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )
)
