(define (problem tidy_delivery_p1)
  (:domain tidy_delivery)
  (:objects
    r1 - robot
    kitchen hall - room
    cup - item
  )
  (:init
    (robot_at r1 kitchen)
    (item_at cup kitchen)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
  )
  (:goal (and
    (robot_at r1 hall)
    (preference cup_in_hall (item_at cup hall))
  ))
  (:metric minimize (is-violated cup_in_hall))
)
