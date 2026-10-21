(define (domain deep_types)
(:requirements :strips :typing :negative-preconditions :fluents :durative-actions)
; Four levels: robot > ground > soft > infantry
(:types
  zone robot - object
  drone ground - robot
  tank soft - ground
  infantry artillery - soft
)
(:predicates
  (at ?r - robot ?z - zone)
  (armored ?g - ground)
  (light ?s - soft)
  (captured ?z - zone)
)
(:functions
  (ammo ?g - ground)
)
(:durative-action move
  :parameters (?r - robot ?from ?to - zone)
  :duration (= ?duration 1)
  :condition (and (at start (at ?r ?from)))
  :effect (and (at start (not (at ?r ?from))) (at end (at ?r ?to))))
(:action capture
  :parameters (?i - infantry ?z - zone)
  :precondition (and (at ?i ?z))
  :effect (and (captured ?z)))
)
