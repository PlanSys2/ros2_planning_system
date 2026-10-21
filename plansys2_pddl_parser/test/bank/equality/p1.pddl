(define (problem equality_p1)
  (:domain equality_handover)
  (:objects
    r1 r2 - robot
    hall - room
    box1 - box
  )
  (:init
    (robot_at r1 hall)
    (robot_at r2 hall)
    (holding r1 box1)
    (gripper_free r2)
    (home r2 hall)
  )
  (:goal (and (received r2 box1) (docked r2)))
)
