(define (problem equality_p2)
  (:domain equality_handover)
  (:objects
    r1 r2 r3 - robot
    kitchen hall office lab - room
    box1 - box
  )
  (:init
    (robot_at r1 kitchen)
    (robot_at r2 office)
    (robot_at r3 lab)
    (holding r1 box1)
    (gripper_free r2)
    (gripper_free r3)
    (home r1 kitchen)
    (home r3 lab)
    (connected kitchen hall)
    (connected hall kitchen)
    (connected hall office)
    (connected office hall)
    (connected office lab)
    (connected lab office)
  )
  (:goal (and (received r3 box1) (docked r3) (docked r1)))
)
