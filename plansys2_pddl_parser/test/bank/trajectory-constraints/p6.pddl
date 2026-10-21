(define (problem timed_patrol_p6)
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
    (at-most-once (robot_at r1 hall))
    (at end (robot_at r1 dock))
  ))
)
