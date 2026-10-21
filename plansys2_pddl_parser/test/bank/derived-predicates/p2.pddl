(define (problem ready_delivery_p2)
  (:domain ready_delivery)
  (:objects
    kobuki - robot
    dock workshop corridor office - room
    parcel letter - package
  )
  (:init
    (robot_at kobuki corridor)
    (connected dock corridor)
    (connected corridor dock)
    (connected workshop corridor)
    (connected corridor workshop)
    (connected office corridor)
    (connected corridor office)
    (charger_in dock)
    (calibration_target_in workshop)
    (package_at parcel dock)
    (package_at letter workshop)
  )
  (:goal (and
    (package_at parcel office)
    (package_at letter office)
  ))
)
