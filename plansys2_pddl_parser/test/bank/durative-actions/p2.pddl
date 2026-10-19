(define (problem delivery_p2)
  (:domain delivery)
  (:objects
    r1 r2 - robot
    kitchen hall office lab - room
    cup book tool - item
  )
  (:init
    (robot_at r1 kitchen)
    (robot_at r2 lab)
    (gripper_free r1)
    (gripper_free r2)
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
    (item_at book office)
    (item_at tool kitchen)
  ))
)
