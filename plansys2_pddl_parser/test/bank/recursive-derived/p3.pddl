(define (problem route_dispatch_p3)
  (:domain route_dispatch)
  (:objects
    summit - robot
    wp1 wp2 wp3 wp4 wp5 - waypoint
    door_a - door
  )
  (:init
    (robot_at summit wp1)
    (corridor wp1 wp2)
    (corridor wp2 wp3)
    (corridor wp4 wp5)
    (link door_a wp4 wp3)
    (door_at door_a wp4)
  )
  (:goal (and
    (robot_at summit wp5)
  ))
)
