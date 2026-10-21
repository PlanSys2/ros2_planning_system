(define (problem negative_goals_p3)
  (:domain negative_goals_cleaning)
  (:objects
    r1 - robot
    hall office garage - room
  )
  (:init
    (robot_at r1 hall)
    (dirty garage)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and (not (dirty garage))))
)
