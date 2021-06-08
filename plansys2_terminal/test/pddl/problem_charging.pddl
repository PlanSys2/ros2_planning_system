(define (problem simple-problem)
  (:domain simple)

  ;; Instantiate the objects.
  (:objects 
    r2d2 - robot
    rm0 rm1 rm2 - room
  )

  (:init
    ;; Define the initial state predicates.
    (robot_at r2d2 rm0)
    (charging_point_at rm0)

    (connected rm0 rm1)
    (connected rm1 rm0)
    (connected rm1 rm2)
    (connected rm2 rm1)
    (battery_low r2d2)
  )

  (:goal (and
    (robot_at r2d2 rm2)
  ))
    
  (:metric 
    minimize (total-time)
  )
)
