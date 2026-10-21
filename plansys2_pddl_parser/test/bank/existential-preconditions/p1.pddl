(define (problem existential_preconditions_p1)
  (:domain existential_preconditions_lights)
  (:objects
    r1 - robot
    hall office - room
  )
  (:init
    (robot_at r1 hall)
    (has_camera r1)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and (photographed office)))
)
