(define (problem recharge_p2)
  (:domain recharge)
  (:objects
    r1 r2 - robot
    dock hall office lab - room
    c1 c2 - charger
  )
  (:init
    (robot_at r1 dock)
    (robot_at r2 hall)
    (= (battery r1) 0)
    (= (battery r2) 20)
    (charger_at c1 dock)
    (charger_at c2 hall)
    (charger_free c1)
    (charger_free c2)
    (connected dock hall)
    (connected hall office)
    (connected office lab)
  )
  (:goal (and
    (robot_at r1 office)
    (robot_at r2 lab)
  ))
)
