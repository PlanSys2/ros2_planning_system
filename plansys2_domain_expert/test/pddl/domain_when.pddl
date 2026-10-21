(define (domain doors)
(:requirements :strips :typing :conditional-effects)
(:types robot door)
(:predicates
  (open ?d - door)
  (locked ?d - door)
  (pushed ?r - robot ?d - door)
)
(:action push
  :parameters (?r - robot ?d - door)
  :precondition (and (not (pushed ?r ?d)))
  :effect (and (pushed ?r ?d) (when (not (locked ?d)) (open ?d)))
)
)
