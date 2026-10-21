(define (domain cleaning)
  (:requirements :strips :typing :conditional-effects :numeric-fluents)
  (:types robot room)
  (:predicates
    (robot_at ?r - robot ?l - room)
    (dirty ?l - room)
    (clean ?l - room)
  )
  (:functions
    (battery ?r - robot)
    (cleaned)
  )
  (:action sweep
    :parameters (?r - robot ?l - room)
    :precondition (and (robot_at ?r ?l))
    :effect (and
      (when (dirty ?l) (and (clean ?l) (not (dirty ?l)) (increase (cleaned) 1)))
      (decrease (battery ?r) 1)
    )
  )
)
