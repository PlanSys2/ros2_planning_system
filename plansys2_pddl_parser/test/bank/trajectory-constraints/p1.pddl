(define (problem constrained_delivery_p1)
  (:domain constrained_delivery)
  (:objects
    r1 - robot
    kitchen hall office - room
    cup - item
  )
  (:init
    (robot_at r1 kitchen)
    (item_at cup kitchen)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and
    (robot_at r1 office)
  ))
  (:constraints (and
    (sometime (carrying r1 cup))
    (at-most-once (robot_at r1 hall))
  ))
)
