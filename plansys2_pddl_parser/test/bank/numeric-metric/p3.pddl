(define (problem energy_aware_p3)
  (:domain energy_aware)
  (:objects
    roomba - robot
    ground first second - room
  )
  (:init
    (robot_at roomba ground)
    (lift ground first)
    (ramp first ground)
    (lift second first)
    (= (energy-used) 0)
  )
  (:goal (and
    (cleaned second)
  ))
  (:metric minimize (energy-used))
)
