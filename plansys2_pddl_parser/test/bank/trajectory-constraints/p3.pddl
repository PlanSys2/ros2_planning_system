(define (problem constrained_delivery_p3)
  (:domain constrained_delivery)
  (:objects
    r1 - robot
    kitchen hall office - room
    cup - item
  )
  (:init
    (robot_at r1 kitchen)
    (item_at cup office)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and
    (item_at cup kitchen)
  ))
  (:constraints (and
    (at-most-once (robot_at r1 hall))
  ))
)
