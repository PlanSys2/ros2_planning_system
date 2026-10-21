(define (problem delivery_p3)
  (:domain delivery)
  (:objects
    r1 - robot
    kitchen hall garage - room
    cup - item
  )
  (:init
    (robot_at r1 kitchen)
    (item_at cup kitchen)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected garage hall)
  )
  (:goal (and
    (item_at cup garage)
  ))
)
