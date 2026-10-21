(define (problem timed_patrol_p4)
  (:domain timed_patrol)
  (:objects
    r1 - robot
    dock hall lab - room
  )
  (:init
    (robot_at r1 dock)
    (connected dock hall)
    (connected hall dock)
    (connected hall lab)
    (connected lab hall)
  )
  (:goal (and
    (visited lab)
  ))
  (:constraints (and
    (within 5 (visited lab))
    (always-within 3 (robot_at r1 hall) (robot_at r1 lab))
    (at end (robot_at r1 lab))
  ))
)
