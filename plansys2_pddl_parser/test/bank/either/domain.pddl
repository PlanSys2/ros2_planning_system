(define (domain either_inspection)
  (:requirements :strips :typing)
  (:types
    robot drone - object
    room corridor - object
  )
  (:predicates
    (agent_at ?a - (either robot drone) ?l - (either room corridor))
    (linked ?from ?to - (either room corridor))
    (inspected ?l - (either room corridor))
  )
  (:action move
    :parameters (?a - (either robot drone) ?from ?to - (either room corridor))
    :precondition (and (agent_at ?a ?from) (linked ?from ?to))
    :effect (and (not (agent_at ?a ?from)) (agent_at ?a ?to))
  )
  (:action inspect
    :parameters (?a - (either robot drone) ?l - (either room corridor))
    :precondition (and (agent_at ?a ?l))
    :effect (and (inspected ?l))
  )
)
