(define (problem inspection_p1)
  (:domain inspection)
  (:objects
    r2d2 - robot
    kitchen corridor - room
  )
  (:init
    (robot_at r2d2 kitchen)
    (inspected corridor)
    (connected kitchen corridor)
    (connected corridor kitchen)
  )
  (:goal (and
    (report_sent r2d2)
  ))
)
