(define (problem p) (:domain quantifiers)
(:objects z1 z2 - zone d1 - drone t1 - tank i1 - infantry e1 - tank)
(:init (at i1 z1) (at t1 z1) (at e1 z2) (hostile e1) (available d1) (available i1))
(:goal (and (forall (?z - zone) (imply (exists (?e - robot) (and (hostile ?e) (at ?e ?z))) (owned ?z))))))
