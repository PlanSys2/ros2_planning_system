(define (domain equality_handover)
  (:requirements :strips :typing :equality :negative-preconditions)
  (:types
    robot room box - object
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?from ?to - room)
    (holding ?r - robot ?b - box)
    (gripper_free ?r - robot)
    (home ?r - robot ?l - room)
    (docked ?r - robot)
    (received ?r - robot ?b - box)
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and (robot_at ?r ?from) (connected ?from ?to) (not (= ?from ?to)))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action handover
    :parameters (?giver ?taker - robot ?b - box ?l - room)
    :precondition (and (not (= ?giver ?taker)) (robot_at ?giver ?l) (robot_at ?taker ?l) (holding ?giver ?b) (gripper_free ?taker))
    :effect (and (not (holding ?giver ?b)) (gripper_free ?giver) (holding ?taker ?b) (received ?taker ?b) (not (gripper_free ?taker)))
  )
  (:action dock
    :parameters (?r - robot ?l ?h - room)
    :precondition (and (robot_at ?r ?l) (home ?r ?h) (= ?l ?h))
    :effect (and (docked ?r))
  )
)
