(define (problem scheduled_charge_p1)
  (:domain scheduled_charge)
  (:objects
    r1 - robot
    dock hall - room
  )
  (:init
    (robot_at r1 dock)
    (connected dock hall)
    (= (battery r1) 0)
    (at 1.5 (= (battery r1) 100))
  )
  (:goal (and
    (robot_at r1 hall)
  ))
)
