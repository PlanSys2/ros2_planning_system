(define (problem inspection_p3)
  (:domain inspection)
  (:objects
    r2d2 - robot
    kitchen corridor storage - room
  )
  (:init
    (robot_at r2d2 kitchen)
    (connected kitchen corridor)
    (connected corridor kitchen)
  )
  (:goal (and
    (report_sent r2d2)
  ))
)
