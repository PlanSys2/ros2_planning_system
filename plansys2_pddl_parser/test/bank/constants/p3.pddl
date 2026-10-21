(define (problem constants_p3)
  (:domain constants_charging)
  (:objects
    r1 - robot
    hall kitchen - room
  )
  (:init
    (robot_at r1 kitchen)
    (connected hall kitchen)
    (connected kitchen hall)
  )
  (:goal (and (cleaned kitchen)))
)
