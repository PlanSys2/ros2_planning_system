(define (problem equality_p3)
  (:domain equality_handover)
  (:objects
    r1 - robot
    kitchen hall - room
    box1 - box
  )
  (:init
    (robot_at r1 kitchen)
    (holding r1 box1)
    (gripper_free r1)
    (connected kitchen hall)
    (connected hall kitchen)
  )
  (:goal (and (received r1 box1)))
)
