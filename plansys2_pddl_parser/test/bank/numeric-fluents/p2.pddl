(define (problem battery_patrol_p2)
  (:domain battery_patrol)
  (:objects
    kobuki - robot
    dock hall lab office - room
  )
  (:init
    (robot_at kobuki hall)
    (connected dock hall)
    (connected hall dock)
    (connected hall lab)
    (connected lab hall)
    (connected lab office)
    (connected office lab)
    (charger_in dock)
    (= (battery kobuki) 15)
  )
  (:goal (and
    (patrolled lab)
    (patrolled office)
    (robot_at kobuki dock)
  ))
)
