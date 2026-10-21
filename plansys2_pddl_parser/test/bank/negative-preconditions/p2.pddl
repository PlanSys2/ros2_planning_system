(define (problem negative_preconditions_p2)
  (:domain negative_preconditions_patrol)
  (:objects
    r1 - robot
    hall office lab kitchen storage - room
  )
  (:init
    (robot_at r1 hall)
    (alarm kitchen)
    (connected hall office)
    (connected office hall)
    (connected hall kitchen)
    (connected kitchen hall)
    (connected office lab)
    (connected lab office)
    (connected kitchen storage)
    (connected storage kitchen)
    (connected lab storage)
    (connected storage lab)
  )
  (:goal (and (visited hall) (visited office) (visited lab) (visited storage)))
)
