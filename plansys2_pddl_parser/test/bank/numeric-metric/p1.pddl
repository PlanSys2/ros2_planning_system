(define (problem energy_aware_p1)
  (:domain energy_aware)
  (:objects
    roomba - robot
    ground first - room
  )
  (:init
    (robot_at roomba ground)
    (lift ground first)
    (lift first ground)
    (= (energy-used) 0)
  )
  (:goal (and
    (cleaned first)
  ))
  (:metric minimize (energy-used))
)
