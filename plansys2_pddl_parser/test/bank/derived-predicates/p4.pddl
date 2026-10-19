(define (problem obstacle_rooms_p4)
  (:domain obstacle_rooms)
  (:objects
    r1 - robot
    dock hall lab - room
  )
  (:init
    (robot_at r1 dock)
    (connected dock hall)
    (connected hall lab)
    (obstacle hall)
    (obstacle lab)
  )
  (:goal (and
    (robot_at r1 lab)
  ))
)
