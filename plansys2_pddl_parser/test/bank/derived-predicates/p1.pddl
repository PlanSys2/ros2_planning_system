(define (problem ready_delivery_p1)
  (:domain ready_delivery)
  (:objects
    kobuki - robot
    dock - room
    parcel - package
  )
  (:init
    (robot_at kobuki dock)
    (charged kobuki)
    (calibration_target_in dock)
    (package_at parcel dock)
  )
  (:goal (and
    (carrying kobuki parcel)
  ))
)
