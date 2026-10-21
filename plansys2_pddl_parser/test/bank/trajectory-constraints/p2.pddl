(define (problem constrained_delivery_p2)
  (:domain constrained_delivery)
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
    (item_at tool kitchen)
  ))
  (:constraints (and
    (always (item_at book office))
    (sometime-before (item_at tool kitchen) (item_at cup office))
    (at-most-once (carrying r1 tool))
    (sometime (robot_at r1 lab))
  ))
)
