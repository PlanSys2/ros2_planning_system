(define (problem tidy_delivery_p2)
  (:domain tidy_delivery)
  (:objects
    r1 - robot
    kitchen hall office lab - room
    cup book tool - item
  )
  (:init
    (robot_at r1 kitchen)
    (gripper_free r1)
    (item_at cup kitchen)
    (item_at book office)
    (item_at tool lab)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected hall office)
    (connected office hall)
    (connected hall lab)
    (connected lab hall)
  )
  (:goal (and
    (item_at cup office)
    (robot_at r1 hall)
    (preference book_in_kitchen (item_at book kitchen))
    (preference tool_in_hall (item_at tool hall))
  ))
  (:metric minimize (+ (* 2 (is-violated book_in_kitchen)) (is-violated tool_in_hall)))
)
