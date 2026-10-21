(define (problem recharge_p1)
  (:domain recharge)
  (:objects
    r1 - robot
    dock hall office - room
    c1 - charger
  )
  (:init
    (robot_at r1 dock)
    (= (battery r1) 10)
    (charger_at c1 dock)
    (charger_free c1)
    (connected dock hall)
    (connected hall office)
  )
  (:goal (and
    (robot_at r1 office)
  ))
)
