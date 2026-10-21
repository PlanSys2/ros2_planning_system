(define (problem continuous_recharge_p3)
  (:domain continuous_recharge)
  (:objects
    r1 - robot
    dock hall office - room
    c1 - charger
  )
  (:init
    (robot_at r1 dock)
    (= (battery r1) 0)
    (charger_at c1 dock)
    (charger_free c1)
    (connected dock hall)
    (connected office hall)
  )
  (:goal (and
    (robot_at r1 office)
  ))
)
