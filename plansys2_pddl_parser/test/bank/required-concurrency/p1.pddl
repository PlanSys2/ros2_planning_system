(define (problem door_holding_p1)
  (:domain door_holding)
  (:objects
    holder walker - robot
    corridor office - room
    office_door - door
  )
  (:init
    (robot_at holder corridor)
    (robot_at walker corridor)
    (door_between office_door corridor office)
    (door_between office_door office corridor)
  )
  (:goal (and
    (robot_at walker office)
  ))
)
