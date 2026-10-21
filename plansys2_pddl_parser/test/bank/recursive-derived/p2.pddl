(define (problem route_dispatch_p2)
  (:domain route_dispatch)
  (:objects
    summit kobuki - robot
    wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 - waypoint
    door_a door_b door_c - door
  )
  (:init
    (robot_at summit wp1)
    (robot_at kobuki wp8)
    (corridor wp1 wp2)
    (corridor wp2 wp3)
    (corridor wp4 wp5)
    (corridor wp5 wp6)
    (corridor wp7 wp8)
    (corridor wp8 wp7)
    (link door_a wp3 wp4)
    (door_at door_a wp3)
    (link door_b wp6 wp7)
    (door_at door_b wp6)
    (link door_c wp7 wp1)
    (door_at door_c wp7)
  )
  (:goal (and
    (robot_at summit wp8)
    (robot_at kobuki wp2)
  ))
)
