(define (problem disjunctive_preconditions_p1)
  (:domain disjunctive_preconditions_doors)
  (:objects
    r1 - robot
    hall office - room
    d1 - door
  )
  (:init
    (robot_at r1 office)
    (door_between d1 hall office)
    (door_open d1)
  )
  (:goal (and (robot_at r1 hall)))
)
