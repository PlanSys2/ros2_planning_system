(define (problem negative_preconditions_p3)
  (:domain negative_preconditions_patrol)
  (:objects
    r1 - robot
    hall office - room
  )
  (:init
    (robot_at r1 hall)
    (alarm office)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and (visited office)))
)
