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
    (robot_at rob1 entrance)
    (= (battery_level bot) 90)
    (= (battery_level rob1) 100)
    (unknown (robot_at bot entrance))
    (connected entrance dinning)
    (unknown (robot_at bot kitchen))
    (oneof (robot_at bot entrance) (robot_at bot kitchen))
)

(:goal (and
        (robot_at bot bathroom)
))


)