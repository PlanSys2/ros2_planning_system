(define (problem powered_doors_p1)
  (:domain powered_doors)
  (:objects
    tiago - robot
    kitchen corridor - room
    kitchen_door - door
    kitchen_button - button
  )
  (:init
    (robot_at tiago kitchen)
    (door_between kitchen_door kitchen corridor)
    (door_between kitchen_door corridor kitchen)
    (button_in kitchen_button kitchen)
    (controls kitchen_button kitchen_door)
    (powered kitchen_button)
  )
  (:goal (and
    (robot_at tiago corridor)
  ))
)
