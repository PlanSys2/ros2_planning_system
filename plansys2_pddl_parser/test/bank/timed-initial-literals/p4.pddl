(define (problem closing_doors_p4)
  (:domain closing_doors)
  (:objects
    r1 - robot
    corridor office - room
    office_door - door
  )
  (:init
    (robot_at r1 corridor)
    (door_between office_door corridor office)
    (at 1.5 (door_open office_door))
  )
  (:goal (and
    (robot_at r1 office)
  ))
)
