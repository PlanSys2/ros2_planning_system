(define (problem closing_doors_p3)
  (:domain closing_doors)
  (:objects
    r1 - robot
    corridor office lab - room
    office_door lab_door - door
  )
  (:init
    (robot_at r1 corridor)
    (door_between office_door corridor office)
    (door_between lab_door corridor lab)
    (at 2 (door_open lab_door))
    (at 10 (not (door_open lab_door)))
  )
  (:goal (and
    (robot_at r1 office)
  ))
)
