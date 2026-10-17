(define (domain heating)
(:requirements :strips :typing :fluents :time)
(:types room)
(:predicates (heating ?r - room))
(:functions (temperature ?r - room))
(:action switch_on
  :parameters (?r - room)
  :precondition (and (heating ?r))
  :effect (and (heating ?r)))
(:event overheat
  :parameters (?r - room)
  :precondition (and (>= (temperature ?r) 30))
  :effect (and (not (heating ?r))))
)
