(define (problem charging_station_p2)
  (:domain charging_station)
  (:objects
    r1 r2 - robot
    dock hall office lab - room
  )
  (:init
    (robot_at r1 dock)
    (robot_at r2 dock)
    (idle r1)
    (idle r2)
    (has_charger dock)
    (connected dock hall)
    (connected hall office)
    (connected office lab)
    (= (battery r1) 0)
    (= (battery r2) 20)
  )
  (:goal (and
    (robot_at r1 lab)
    (robot_at r2 hall)
  ))
)
