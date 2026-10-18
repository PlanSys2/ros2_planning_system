(define (problem either_p3)
  (:domain either_inspection)
  (:objects
    d1 - drone
    lab vault - room
    north - corridor
  )
  (:init
    (agent_at d1 lab)
    (linked lab north)
    (linked north lab)
  )
  (:goal (and (inspected vault)))
)
