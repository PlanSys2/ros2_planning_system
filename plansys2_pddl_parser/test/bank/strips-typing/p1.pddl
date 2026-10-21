(define (problem strips_typing_p1)
  (:domain strips_typing_delivery)
  (:objects
    r1 - robot
    kitchen hall - room
    box1 - box
  )
  (:init
    (robot_at r1 kitchen)
    (box_at box1 kitchen)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
  )
  (:goal (and (box_at box1 hall)))
)
