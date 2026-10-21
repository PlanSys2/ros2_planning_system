(define (problem constants_p2)
  (:domain constants_charging)
  (:objects
    r1 - robot
    hall kitchen office - room
  )
  (:init
    (robot_at r1 kitchen)
    (connected hall charging_station)
    (connected charging_station hall)
    (connected hall kitchen)
    (connected kitchen hall)
    (connected hall office)
    (connected office hall)
  )
  (:goal (and (cleaned kitchen) (cleaned office) (robot_at r1 charging_station)))
)
