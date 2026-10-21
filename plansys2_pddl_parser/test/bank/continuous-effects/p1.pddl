(define (problem continuous_recharge_p1)
  (:domain continuous_recharge)
  (:objects
    r1 - robot
    dock hall - room
    c1 - charger
  )
  (:init
    (robot_at r1 dock)
    (= (battery r1) 0)
    (charger_at c1 dock)
    (charger_free c1)
    (connected dock hall)
  )
  (:goal (and
    (robot_at r1 hall)
  ))
)
