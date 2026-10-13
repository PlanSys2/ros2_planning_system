; Sliding tile puzzle
(define (domain puzzle)
  (:requirements :strips :typing)
  (:types tile position)
  (:predicates (at ?t - tile ?p - position) (blank ?p - position) (adjacent ?a ?b - position))
  (:action slide
    :parameters (?t - tile ?from ?to - position)
    :precondition (and (at ?t ?from) (blank ?to) (adjacent ?from ?to))
    :effect (and (not (at ?t ?from)) (not (blank ?to)) (at ?t ?to) (blank ?from))))
