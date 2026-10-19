(define (domain heating)
(:requirements :strips :typing :fluents :time)
(:types room)
(:predicates (heating ?r - room))
(:functions (temperature ?r - room))
(:action switch_on
  :parameters (?r - room)
  :precondition (and (heating ?r))
  :effect (and (heating ?r)))
(:process heat
  :parameters (?r - room)
  :precondition (and (heating ?r))
  :effect (and (increase (temperature ?r) (* #t 1.0))))
)
