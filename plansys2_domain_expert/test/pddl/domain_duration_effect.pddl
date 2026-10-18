(define (domain charging)
(:requirements :strips :typing :durative-actions :numeric-fluents :duration-inequalities)
(:types robot)
(:predicates
  (docked ?r - robot)
)
(:functions
  (battery ?r - robot)
)
(:durative-action charge
  :parameters (?r - robot)
  :duration (and (>= ?duration 1) (<= ?duration 8))
  :condition (and (over all (docked ?r)))
  :effect (and (at end (increase (battery ?r) (* 10 ?duration))))
)
)
