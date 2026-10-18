(define (problem either_p2)
  (:domain either_inspection)
  (:objects
    r1 - robot
    d1 - drone
    lab office storage - room
    north south - corridor
  )
  (:init
    (agent_at r1 lab)
    (agent_at d1 storage)
    (linked lab north)
    (linked north lab)
    (linked north office)
    (linked office north)
    (linked office south)
    (linked south office)
    (linked south storage)
    (linked storage south)
  )
  (:goal (and (inspected lab) (inspected north) (inspected office) (inspected south) (inspected storage)))
)
