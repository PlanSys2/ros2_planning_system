(define (problem existential_preconditions_p2)
  (:domain existential_preconditions_lights)
  (:objects
    r1 r2 - robot
    hall office lab kitchen - room
  )
  (:init
    (robot_at r1 hall)
    (robot_at r2 kitchen)
    (has_camera r2)
    (connected hall office)
    (connected office hall)
    (connected office lab)
    (connected lab office)
    (connected hall kitchen)
    (connected kitchen hall)
  )
  (:goal (and (photographed lab) (photographed office) (lit hall)))
)
