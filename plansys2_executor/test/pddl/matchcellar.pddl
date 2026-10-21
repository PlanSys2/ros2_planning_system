; MatchCellar (IPC 2011, temporal): a fuse can only be mended while a match is lit,
; so mend_fuse must happen during light_match (required concurrency)
(define (domain matchcellar)
  (:requirements :typing :durative-actions)
  (:types match fuse)
  (:predicates
    (handfree)
    (unused ?match - match)
    (mended ?fuse - fuse)
    (light ?match - match))

  (:durative-action light_match
    :parameters (?match - match)
    :duration (= ?duration 5)
    :condition (and
      (at start (unused ?match)))
    :effect (and
      (at start (not (unused ?match)))
      (at start (light ?match))
      (at end (not (light ?match)))))

  (:durative-action mend_fuse
    :parameters (?fuse - fuse ?match - match)
    :duration (= ?duration 2)
    :condition (and
      (at start (handfree))
      (over all (light ?match)))
    :effect (and
      (at start (not (handfree)))
      (at end (mended ?fuse))
      (at end (handfree))))
)
