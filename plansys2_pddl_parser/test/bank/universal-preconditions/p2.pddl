(define (problem inspection_p2)
  (:domain inspection)
  (:objects
    r2d2 - robot
    kitchen corridor livingroom bedroom bathroom - room
  )
  (:init
    (robot_at r2d2 kitchen)
    (connected kitchen corridor)
    (connected corridor kitchen)
    (connected corridor livingroom)
    (connected livingroom corridor)
    (connected corridor bedroom)
    (connected bedroom corridor)
    (connected bedroom bathroom)
    (connected bathroom bedroom)
  )
  (:goal (and
    (report_sent r2d2)
  ))
)
