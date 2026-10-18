(define (problem constants_p1)
  (:domain constants_charging)
  (:objects
    r1 - robot
    hall - room
  )
  (:init
    (robot_at r1 hall)
    (connected hall charging_station)
    (connected charging_station hall)
  )
  (:goal (and (charged r1) (robot_at r1 charging_station)))
)
