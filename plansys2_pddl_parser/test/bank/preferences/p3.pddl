(define (problem tidy_delivery_p3)
  (:domain tidy_delivery)
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
    (robot_at r1 garage)
    (preference cup_in_hall (item_at cup hall))
  ))
  (:metric minimize (is-violated cup_in_hall))
)
