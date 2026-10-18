(define (domain door_holding)
  (:requirements :strips :typing :durative-actions)
  (:types
    robot room door
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (door_between ?d - door ?a - room ?b - room)
    (door_open ?d - door)
  )
  (:durative-action hold_door
    :parameters (?r - robot ?d - door ?l - room ?other - room)
    :duration (= ?duration 10)
    :condition (and
      (over all (robot_at ?r ?l))
      (over all (door_between ?d ?l ?other))
    )
    :effect (and
      (at start (door_open ?d))
      (at end (not (door_open ?d)))
    )
  )
  (:durative-action pass_door
    :parameters (?r - robot ?d - door ?from - room ?to - room)
    :duration (= ?duration 4)
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (door_between ?d ?from ?to))
      (over all (door_open ?d))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )
)
