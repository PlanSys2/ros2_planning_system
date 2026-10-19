(define (domain doors)
(:requirements :strips :typing :preferences)
(:types robot door)
(:predicates
  (open ?d - door)
  (near ?r - robot ?d - door)
)
(:action open_door
  :parameters (?r - robot ?d - door)
  :precondition (and (preference close (near ?r ?d)))
  :effect (and (open ?d))
)
)
