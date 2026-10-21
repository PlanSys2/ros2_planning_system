(define (domain delivery)
  (:requirements :strips :typing :durative-actions)
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
  (:durative-action move
    :parameters (?r - robot ?from - room ?to - room)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )
  (:durative-action pick
    :parameters (?r - robot ?i - item ?l - room)
    :duration (= ?duration 2)
    :condition (and
      (at start (item_at ?i ?l))
      (at start (gripper_free ?r))
      (over all (robot_at ?r ?l))
    )
    :effect (and
      (at start (not (item_at ?i ?l)))
      (at start (not (gripper_free ?r)))
      (at end (carrying ?r ?i))
    )
  )
  (:durative-action drop
    :parameters (?r - robot ?i - item ?l - room)
    :duration (= ?duration 2)
    :condition (and
      (at start (carrying ?r ?i))
      (over all (robot_at ?r ?l))
    )
    :effect (and
      (at start (not (carrying ?r ?i)))
      (at end (item_at ?i ?l))
      (at end (gripper_free ?r))
    )
  )
)
