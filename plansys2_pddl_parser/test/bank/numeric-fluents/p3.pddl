(define (problem battery_patrol_p3)
  (:domain battery_patrol)
  (:objects
    kobuki - robot
    dock hall lab - room
  )
  (:init
    (robot_at kobuki dock)
    (connected dock hall)
    (connected hall dock)
    (connected hall lab)
    (connected lab hall)
    (= (battery kobuki) 24)
  )
  (:goal (and
    (patrolled lab)
  ))
)
