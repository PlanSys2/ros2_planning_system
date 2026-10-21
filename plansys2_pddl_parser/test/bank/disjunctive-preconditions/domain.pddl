(define (domain disjunctive_preconditions_doors)
  (:requirements :strips :typing :disjunctive-preconditions)
  (:types
    robot room door - object
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (door_between ?d - door ?from ?to - room)
    (door_open ?d - door)
    (has_badge ?r - robot)
    (badge_at ?l - room)
  )
  (:action pass_door
    :parameters (?r - robot ?d - door ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (or (door_between ?d ?from ?to) (door_between ?d ?to ?from))
      (or (door_open ?d) (has_badge ?r)))
    :effect (and (not (robot_at ?r ?from)) (robot_at ?r ?to))
  )
  (:action take_badge
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l) (badge_at ?l))
    :effect (and (has_badge ?r))
  )
)
