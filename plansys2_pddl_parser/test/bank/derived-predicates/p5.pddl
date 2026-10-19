(define (problem obstacle_rooms_p5)
  (:domain obstacle_rooms)
  (:objects
    r1 - robot
    dock hall lab - room
  )
  (:init
    (robot_at r1 dock)
    (connected dock hall)
    (obstacle hall)
    (obstacle lab)
  )
  (:goal (and
    (robot_at r1 lab)
  ))
)
