(define (domain object_positions)
  (:requirements :strips :typing :equality :object-fluents)
  (:types
    robot room box
  )
  (:predicates
    (connected ?from ?to - room)
  )
  (:functions
    (pos ?r - robot) - room
    (box_pos ?b - box) - room
  )
  (:action move
    :parameters (?r - robot ?from ?to - room)
    :precondition (and
      (= (pos ?r) ?from)
      (connected ?from ?to)
    )
    :effect (and
      (assign (pos ?r) ?to)
    )
  )
  (:action push
    :parameters (?r - robot ?b - box ?from ?to - room)
    :precondition (and
      (= (pos ?r) ?from)
      (= (box_pos ?b) ?from)
      (connected ?from ?to)
    )
    :effect (and
      (assign (pos ?r) ?to)
      (assign (box_pos ?b) ?to)
    )
  )
)
