(define (domain constants_charging)
  (:requirements :strips :typing)
  (:types
    robot room - object
  )
  (:constants
    charging_station - room
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (charged ?r - robot)
    (cleaned ?l - room)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action charge
    :parameters (?r - robot)
    :precondition (and (robot_at ?r charging_station))
    :effect (and (charged ?r))
  )
  (:action clean
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l) (charged ?r))
    :effect (and (cleaned ?l) (not (charged ?r)))
  )
)
