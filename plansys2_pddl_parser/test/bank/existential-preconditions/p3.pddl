(define (problem existential_preconditions_p3)
  (:domain existential_preconditions_lights)
  (:objects
    r1 r2 - robot
    hall office - room
  )
  (:init
    (robot_at r1 hall)
    (robot_at r2 office)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and (photographed office)))
)
