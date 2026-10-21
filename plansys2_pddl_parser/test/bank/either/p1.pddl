(define (problem either_p1)
  (:domain either_inspection)
  (:objects
    r1 - robot
    lab - room
    north - corridor
  )
  (:init
    (agent_at r1 lab)
    (linked lab north)
    (linked north lab)
  )
  (:goal (and (inspected north)))
)
