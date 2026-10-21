(define (problem charging_station_p3)
  (:domain charging_station)
  (:objects
    r1 - robot
    dock hall office - room
  )
  (:init
    (robot_at r1 dock)
    (idle r1)
    (has_charger dock)
    (connected dock hall)
    (connected office hall)
    (= (battery r1) 0)
  )
  (:goal (and
    (robot_at r1 office)
  ))
)
