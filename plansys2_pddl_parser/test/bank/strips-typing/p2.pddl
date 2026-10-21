(define (problem strips_typing_p2)
  (:domain strips_typing_delivery)
  (:objects
    r1 - robot
    kitchen hall office lab - room
    box1 box2 - box
  )
  (:init
    (robot_at r1 kitchen)
    (box_at box1 kitchen)
    (box_at box2 lab)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected hall office)
    (connected office hall)
    (connected office lab)
    (connected lab office)
  )
  (:goal (and (box_at box1 lab) (box_at box2 kitchen)))
)
