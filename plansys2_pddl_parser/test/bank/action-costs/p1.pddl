(define (problem costly_delivery_p1)
  (:domain costly_delivery)
  (:objects
    kobuki - robot
    dock office - room
    parcel - package
  )
  (:init
    (robot_at kobuki dock)
    (package_at parcel dock)
    (connected dock office)
    (connected office dock)
    (= (distance dock office) 4)
    (= (distance office dock) 4)
    (= (total-cost) 0)
  )
  (:goal (and
    (package_at parcel office)
  ))
  (:metric minimize (total-cost))
)
