(define (problem charging_station_p1)
  (:domain charging_station)
  (:objects
    r1 - robot
    dock hall - room
  )
  (:init
    (robot_at r1 dock)
    (idle r1)
    (has_charger dock)
    (connected dock hall)
    (= (battery r1) 0)
  )
  (:goal (and
    (robot_at r1 hall)
  ))
)
