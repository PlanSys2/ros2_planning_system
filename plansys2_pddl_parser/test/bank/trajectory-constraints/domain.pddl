(define (domain constrained_delivery)
  (:requirements :strips :typing :constraints)
  (:types
    robot room item
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (item_at ?i - item ?l - room)
    (connected ?a - room ?b - room)
    (carrying ?r - robot ?i - item)
    (gripper_free ?r - robot)
  )
  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )
  (:action pick
    :parameters (?r - robot ?i - item ?l - room)
    :precondition (and
      (robot_at ?r ?l)
      (item_at ?i ?l)
      (gripper_free ?r)
    )
    :effect (and
      (not (item_at ?i ?l))
      (not (gripper_free ?r))
      (carrying ?r ?i)
    )
  )
  (:action drop
    :parameters (?r - robot ?i - item ?l - room)
    :precondition (and
      (robot_at ?r ?l)
      (carrying ?r ?i)
    )
    :effect (and
      (not (carrying ?r ?i))
      (item_at ?i ?l)
      (gripper_free ?r)
    )
  )
)
