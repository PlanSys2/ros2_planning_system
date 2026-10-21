(define (problem door_holding_p2)
  (:domain door_holding)
  (:objects
    r1 r2 r3 - robot
    corridor office lab - room
    office_door lab_door - door
  )
  (:init
    (robot_at r1 corridor)
    (robot_at r2 corridor)
    (robot_at r3 office)
    (door_between office_door corridor office)
    (door_between office_door office corridor)
    (door_between lab_door office lab)
    (door_between lab_door lab office)
  )
  (:goal (and
    (robot_at r1 lab)
    (robot_at r3 corridor)
  ))
)
