(define (problem disjunctive_preconditions_p2)
  (:domain disjunctive_preconditions_doors)
  (:objects
    r1 - robot
    hall office lab storage vault - room
    d1 d2 d3 d4 - door
  )
  (:init
    (robot_at r1 office)
    (door_between d1 hall office)
    (door_between d2 office lab)
    (door_between d3 lab storage)
    (door_between d4 hall vault)
    (door_open d1)
    (door_open d2)
    (door_open d3)
    (badge_at storage)
  )
  (:goal (and (robot_at r1 vault)))
)
