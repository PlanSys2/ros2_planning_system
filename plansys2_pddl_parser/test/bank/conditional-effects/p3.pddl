(define (problem powered_doors_p3)
  (:domain powered_doors)
  (:objects
    tiago - robot
    kitchen corridor - room
    kitchen_door - door
    kitchen_button - button
    main_panel - panel
  )
  (:init
    (robot_at tiago kitchen)
    (door_between kitchen_door kitchen corridor)
    (door_between kitchen_door corridor kitchen)
    (button_in kitchen_button kitchen)
    (controls kitchen_button kitchen_door)
    (panel_in main_panel corridor)
    (feeds main_panel kitchen_button)
  )
  (:goal (and
    (robot_at tiago corridor)
  ))
)
