(define (problem timed_patrol_p5)
  (:domain timed_patrol)
  (:objects
    r1 - robot
    dock hall lab - room
  )
  (:init
    (robot_at r1 dock)
    (connected dock hall)
    (connected hall dock)
    (connected hall lab)
    (connected lab hall)
  )
  (:goal (and
    (lights_on lab)
  ))
  (:constraints (and
    (hold-during 3 5 (visited hall))
    (hold-after 6 (lights_on lab))
    (sometime-after (robot_at r1 hall) (lights_on lab))
  ))
)
