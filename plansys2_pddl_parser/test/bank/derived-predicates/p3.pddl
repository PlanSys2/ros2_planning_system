(define (problem ready_delivery_p3)
  (:domain ready_delivery)
  (:objects
    kobuki - robot
    dock corridor office - room
    parcel - package
  )
  (:init
    (robot_at kobuki corridor)
    (connected dock corridor)
    (connected corridor dock)
    (connected office corridor)
    (connected corridor office)
    (charger_in dock)
    (package_at parcel dock)
  )
  (:goal (and
    (package_at parcel office)
  ))
)
