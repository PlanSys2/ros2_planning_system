(define (problem negative_goals_p1)
  (:domain negative_goals_cleaning)
  (:objects
    r1 - robot
    hall office - room
  )
  (:init
    (robot_at r1 hall)
    (dirty office)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and (not (dirty office)) (not (robot_at r1 office))))
)
