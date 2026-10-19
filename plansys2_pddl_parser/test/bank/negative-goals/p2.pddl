(define (problem negative_goals_p2)
  (:domain negative_goals_cleaning)
  (:objects
    r1 - robot
    hall office lab kitchen - room
  )
  (:init
    (robot_at r1 hall)
    (dirty office)
    (dirty lab)
    (dirty kitchen)
    (connected hall office)
    (connected office hall)
    (connected office lab)
    (connected lab office)
    (connected hall kitchen)
    (connected kitchen hall)
  )
  (:goal (and (not (dirty office)) (not (dirty lab)) (not (dirty kitchen)) (robot_at r1 hall)))
)
