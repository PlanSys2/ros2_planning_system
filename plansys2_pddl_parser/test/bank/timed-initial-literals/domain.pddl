(define (domain closing_doors)
  (:requirements :strips :typing :durative-actions :timed-initial-literals)
  (:types
    robot room door
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (door_between ?d - door ?a - room ?b - room)
    (door_open ?d - door)
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
