(define (problem test) (:domain simple)
(:objects
    bot - robot
  entrance  - room
  kitchen - room
  bedroom - room
  dinning - room
  bathroom - room
  chargingroom - room
)

(:init
    (connected entrance dinning)
    (connected dinning entrance)
    (connected dinning kitchen)
    (connected kitchen dinning)
    (connected dinning bedroom)
    (connected bedroom dinning)
    (connected bathroom bedroom)
    (connected bedroom bathroom)
    (connected chargingroom kitchen)
    (connected kitchen chargingroom)
    (charging_point_at chargingroom)
    (robot_at bot entrance)
    (robot_at rob1 entrance)
    (= (battery_level bot) 90)
    (= (battery_level rob1) 100)
)

(:goal (and
        (robot_at bot bathroom)
))


)