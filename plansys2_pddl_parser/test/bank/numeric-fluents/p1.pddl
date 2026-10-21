(define (problem battery_patrol_p1)
  (:domain battery_patrol)
  (:objects
    kobuki - robot
    dock hall - room
  )
  (:init
    (robot_at kobuki dock)
    (connected dock hall)
    (connected hall dock)
    (= (battery kobuki) 20)
  )
  (:goal (and
    (patrolled hall)
  ))
)
