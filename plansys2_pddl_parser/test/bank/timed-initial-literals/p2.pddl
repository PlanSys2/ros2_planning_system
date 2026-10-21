(define (problem closing_doors_p2)
  (:domain closing_doors)
  (:objects
    r1 - robot
    corridor office lab - room
    office_door lab_door - door
  )
  (:init
    (robot_at r1 corridor)
    (door_open office_door)
    (door_open lab_door)
    (door_between office_door corridor office)
    (door_between lab_door office lab)
    (at 7 (not (door_open office_door)))
    (at 12 (not (door_open lab_door)))
  )
  (:goal (and
    (robot_at r1 lab)
  ))
)
