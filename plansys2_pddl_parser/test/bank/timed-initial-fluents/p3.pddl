(define (problem scheduled_charge_p3)
  (:domain scheduled_charge)
  (:objects
    r1 - robot
    dock hall lab - room
  )
  (:init
    (robot_at r1 dock)
    (connected dock hall)
    (= (battery r1) 0)
    (at 1 (= (battery r1) 100))
  )
  (:goal (and
    (robot_at r1 lab)
  ))
)
