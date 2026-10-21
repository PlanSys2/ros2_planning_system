(define (problem disjunctive_preconditions_p3)
  (:domain disjunctive_preconditions_doors)
  (:objects
    r1 - robot
    hall vault - room
    d1 - door
  )
  (:init
    (robot_at r1 hall)
    (door_between d1 hall vault)
  )
  (:goal (and (robot_at r1 vault)))
)
