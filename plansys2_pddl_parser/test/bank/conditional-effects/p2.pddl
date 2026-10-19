(define (problem powered_doors_p2)
  (:domain powered_doors)
  (:objects
    tiago - robot
    kitchen corridor lab office - room
    kitchen_door lab_door office_door - door
    kitchen_button lab_button office_button - button
    main_panel lab_panel - panel
  )
  (:init
    (robot_at tiago kitchen)
    (door_between kitchen_door kitchen corridor)
    (door_between kitchen_door corridor kitchen)
    (door_between lab_door corridor lab)
    (door_between lab_door lab corridor)
    (door_between office_door lab office)
    (door_between office_door office lab)
    (button_in kitchen_button kitchen)
    (button_in lab_button corridor)
    (button_in office_button lab)
    (controls kitchen_button kitchen_door)
    (controls lab_button lab_door)
    (controls office_button office_door)
    (panel_in main_panel kitchen)
    (panel_in lab_panel lab)
    (feeds main_panel kitchen_button)
    (feeds main_panel lab_button)
    (feeds lab_panel office_button)
  )
  (:goal (and
    (robot_at tiago office)
  ))
)
