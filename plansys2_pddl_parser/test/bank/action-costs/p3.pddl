(define (problem costly_delivery_p3)
  (:domain costly_delivery)
  (:objects
    kobuki - robot
    dock hall office - room
    parcel - package
  )
  (:init
    (robot_at kobuki dock)
    (package_at parcel dock)
    (connected dock hall)
    (connected hall dock)
    (= (distance dock hall) 3)
    (= (distance hall dock) 3)
    (= (total-cost) 0)
  )
  (:goal (and
    (package_at parcel office)
  ))
  (:metric minimize (total-cost))
)
