(define (problem door_holding_p3)
  (:domain door_holding)
  (:objects
    walker - robot
    corridor office - room
    office_door - door
  )
  (:init
    (robot_at walker corridor)
    (door_between office_door corridor office)
    (door_between office_door office corridor)
  )
  (:goal (and
    (robot_at walker office)
  ))
)
