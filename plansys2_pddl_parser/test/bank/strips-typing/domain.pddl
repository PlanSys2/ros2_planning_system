(define (domain strips_typing_delivery)
  (:requirements :strips :typing)
  (:types
    robot room box - object
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (box_at ?b - box ?l - room)
    (connected ?from ?to - room)
    (gripper_free ?r - robot)
    (holding ?r - robot ?b - box)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action pick
    :parameters (?r - robot ?b - box ?l - room)
    :precondition (and (robot_at ?r ?l) (box_at ?b ?l) (gripper_free ?r))
    :effect (and (holding ?r ?b) (not (box_at ?b ?l)) (not (gripper_free ?r)))
  )
  (:action drop
    :parameters (?r - robot ?b - box ?l - room)
    :precondition (and (robot_at ?r ?l) (holding ?r ?b))
    :effect (and (box_at ?b ?l) (gripper_free ?r) (not (holding ?r ?b)))
  )
)
