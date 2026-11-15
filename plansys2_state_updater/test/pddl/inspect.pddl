(define (domain state_updater_inspect)
  (:requirements :typing :fluents :durative-actions :negative-preconditions :continuous-effects)
  (:types area)
  (:predicates
    (leak_detected)
    (inspected ?a - area)
  )
  (:functions
    (energy)
  )
  ; Spends energy while it runs (#t), and is cut if a leak shows up (over all)
  (:durative-action inspect
    :parameters (?a - area)
    :duration (= ?duration 30)
    :condition (and (at start (>= (energy) 20)) (over all (not (leak_detected))))
    :effect (and (decrease (energy) (* #t 0.5)) (at end (inspected ?a)))
  )
)
