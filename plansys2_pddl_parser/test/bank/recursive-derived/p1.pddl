(define (problem route_dispatch_p1)
  (:domain route_dispatch)
  (:objects
    summit - robot
    wp1 wp2 wp3 wp4 - waypoint
  )
  (:init
    (robot_at summit wp1)
    (corridor wp1 wp2)
    (corridor wp2 wp3)
    (corridor wp3 wp4)
  )
  (:goal (and
    (robot_at summit wp4)
  ))
)
