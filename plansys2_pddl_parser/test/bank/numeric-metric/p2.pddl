(define (problem energy_aware_p2)
  (:domain energy_aware)
  (:objects
    roomba - robot
    ground landing1 first landing2 second - room
  )
  (:init
    (robot_at roomba ground)
    (lift ground first)
    (lift first ground)
    (lift first second)
    (lift second first)
    (ramp ground landing1)
    (ramp landing1 ground)
    (ramp landing1 first)
    (ramp first landing1)
    (ramp first landing2)
    (ramp landing2 first)
    (ramp landing2 second)
    (ramp second landing2)
    (= (energy-used) 0)
  )
  (:goal (and
    (cleaned first)
    (cleaned second)
    (robot_at roomba ground)
  ))
  (:metric minimize (energy-used))
)
