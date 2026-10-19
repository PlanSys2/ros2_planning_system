(define (problem scheduled_charge_p2)
  (:domain scheduled_charge)
  (:objects
    r1 r2 - robot
    dock hall lab office - room
  )
  (:init
    (robot_at r1 dock)
    (robot_at r2 dock)
    (connected dock hall)
    (connected hall lab)
    (connected lab office)
    (= (battery r1) 100)
    (= (battery r2) 0)
    (at 1 (= (battery r2) 100))
  )
  (:goal (and
    (robot_at r1 lab)
    (robot_at r2 hall)
  ))
)
