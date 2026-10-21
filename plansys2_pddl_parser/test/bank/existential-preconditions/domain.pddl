(define (domain existential_preconditions_lights)
  (:requirements :strips :typing :existential-preconditions)
  (:types
    robot room - object
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (lit ?l - room)
    (has_camera ?r - robot)
    (photographed ?l - room)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action switch_light
    :parameters (?l - room)
    :precondition (and (exists (?r - robot) (robot_at ?r ?l)))
    :effect (and (lit ?l))
  )
  (:action photograph
    :parameters (?l - room)
    :precondition (and (lit ?l) (exists (?r - robot) (and (robot_at ?r ?l) (has_camera ?r))))
    :effect (and (photographed ?l))
  )
)
