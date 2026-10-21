(define (domain quantifiers)
(:requirements :strips :typing :negative-preconditions :existential-preconditions
  :universal-preconditions :durative-actions)
; forall, exists and imply in conditions and effects, over a hierarchy of types
(:types
  zone robot - object
  drone ground - robot
  tank infantry - ground
)
(:predicates
  (at ?r - robot ?z - zone)
  (hostile ?r - robot)
  (available ?r - robot)
  (owned ?z - zone)
)
(:durative-action capture
  :parameters (?r - infantry ?z - zone)
  :duration (= ?duration 5)
  :condition (and
    (at start (forall (?e - robot) (imply (hostile ?e) (not (at ?e ?z)))))
    (over all (at ?r ?z)))
  :effect (and (at end (owned ?z))))
(:durative-action secure
  :parameters (?z - zone)
  :duration (= ?duration 1)
  :condition (and (at start (exists (?g - ground) (at ?g ?z))))
  :effect (and (at end (forall (?r - robot) (not (available ?r))))))
)
