(define (problem charging-problem)
  (:domain charging)

  ;; Instantiate the objects.
  (:objects 
    r2d2 - robot
    wp_control - waypoint
    wp1 wp2 wp3 wp4 - waypoint
  )

  (:init
    ; Define the initial state predicates.
    (robot_at r2d2 wp_control)
    (charger_at wp3)

    (connected wp_control wp1)
    (connected wp1 wp_control)
    (connected wp_control wp2)
    (connected wp2 wp_control)
    (connected wp_control wp3)
    (connected wp3 wp_control)
    (connected wp_control wp4)
    (connected wp4 wp_control)

    ; Define static functions
    (= (speed r2d2) 3)
    (= (max_range r2d2) 75)
    (= (state_of_charge r2d2) 99)

    (= (distance wp1 wp2) 30)
    (= (distance wp1 wp3) 35)
    (= (distance wp1 wp4) 40)
    (= (distance wp1 wp_control) 45)
    (= (distance wp_control wp1) 45)
    (= (distance wp4 wp1) 40)
    (= (distance wp3 wp1) 35)
    (= (distance wp2 wp1) 30)

    (= (distance wp2 wp3) 45)
    (= (distance wp2 wp4) 35)
    (= (distance wp2 wp_control) 30)
    (= (distance wp_control wp2) 30)
    (= (distance wp4 wp2) 35)
    (= (distance wp3 wp2) 45)

    (= (distance wp3 wp4) 40)
    (= (distance wp3 wp_control) 45)
    (= (distance wp_control wp3) 45)
    (= (distance wp4 wp3) 40)

    (= (distance wp4 wp_control) 40)
    (= (distance wp_control wp4) 40)
  )

  (:goal (and
    (patrolled wp1)
    (patrolled wp2)
    (patrolled wp3)
    (patrolled wp4)
  ))
    
  (:metric 
    minimize (total-time)
  )
)
