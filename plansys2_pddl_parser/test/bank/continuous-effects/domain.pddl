(define (domain continuous_recharge)
  (:requirements :strips :typing :durative-actions :numeric-fluents :continuous-effects)
  (:types
    robot room charger
  )
  (:predicates
    (robot_at ?r - robot ?l - room)
    (connected ?a - room ?b - room)
    (charger_at ?c - charger ?l - room)
    (charger_free ?c - charger)
  )
  (:functions
    (battery ?r - robot)
  )
  (:durative-action move
    :parameters (?r - robot ?from - room ?to - room)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (>= (battery ?r) 30))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at start (decrease (battery ?r) 30))
      (at end (robot_at ?r ?to))
    )
  )
  (:durative-action charge
    :parameters (?r - robot ?c - charger ?l - room)
    :duration (= ?duration 4)
    :condition (and
      (at start (charger_free ?c))
      (over all (charger_at ?c ?l))
      (over all (robot_at ?r ?l))
    )
    :effect (and
      (at start (not (charger_free ?c)))
      (at end (charger_free ?c))
      (increase (battery ?r) (* #t 10))
    )
  )
)
