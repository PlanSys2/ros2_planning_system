; For the STNBTBuilder unit tests: durative actions with numeric conditions and effects, a
; non-durative action, and MatchCellar (an action that must happen during another one, and a
; single free hand two actions compete for)
(define (domain stn_test)
  (:requirements :typing :durative-actions :fluents)
  (:types robot room match fuse)
  (:predicates
    (robot_at ?r - robot ?z - room)
    (connected ?z1 ?z2 - room)
    (light_on ?z - room)
    (handfree)
    (unused ?m - match)
    (light ?m - match)
    (mended ?f - fuse))
  (:functions
    (battery ?r - robot))

  (:durative-action move
    :parameters (?r - robot ?z1 ?z2 - room)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?z1))
      (at start (connected ?z1 ?z2))
      (at start (> (battery ?r) 10)))
    :effect (and
      (at start (not (robot_at ?r ?z1)))
      (at end (robot_at ?r ?z2))
      (at end (decrease (battery ?r) 10))))

  (:action switch_on
    :parameters (?r - robot ?z - room)
    :precondition (and (robot_at ?r ?z))
    :effect (and (light_on ?z)))

  (:durative-action light_match
    :parameters (?m - match)
    :duration (= ?duration 5)
    :condition (and (at start (unused ?m)))
    :effect (and
      (at start (not (unused ?m)))
      (at start (light ?m))
      (at end (not (light ?m)))))

  (:durative-action mend_fuse
    :parameters (?f - fuse ?m - match)
    :duration (= ?duration 2)
    :condition (and
      (at start (handfree))
      (over all (light ?m)))
    :effect (and
      (at start (not (handfree)))
      (at end (mended ?f))
      (at end (handfree))))
)
