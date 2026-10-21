(define (problem continuous_recharge_p2)
  (:domain continuous_recharge)
  (:objects
    r1 r2 - robot
    dock hall office - room
    c1 - charger
  )
  (:init
    (robot_at r1 dock)
    (robot_at r2 dock)
    (= (battery r1) 0)
    (= (battery r2) 10)
    (charger_at c1 dock)
    (charger_free c1)
    (connected dock hall)
    (connected hall office)
  )
  (:goal (and
    (robot_at r1 office)
    (robot_at r2 hall)
  ))
)
