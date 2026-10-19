(define (problem tidy_delivery_p4)
  (:domain tidy_delivery)
  (:objects
    r1 - robot
    kitchen hall office - room
    cup book - item
  )
  (:init
    (robot_at r1 kitchen)
    (item_at cup kitchen)
    (item_at book hall)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and
    (item_at cup office)
    (preference book_too (item_at book office))
  ))
  (:constraints (and
    (preference one_pass (at-most-once (robot_at r1 hall)))
    (always (not (item_at cup hall)))
  ))
  (:metric minimize (+ (* 3 (is-violated book_too)) (is-violated one_pass)))
)
