(define (problem elevators-problem)
  (:domain temporal-elevators)

  ;; Instantiate the objects.
  (:objects
    n1 n2 n3 n4 n5 - num
    p1 p2 p3 - passenger
    e1 e2 - elevator
  )

  (:init
    ; Same fluents as the classical planning example
    (next n1 n2)
    (next n2 n3)
    (next n3 n4)
    (next n4 n5)

    (lift_at e1 n1)
    (lift_at e2 n5)

    (passenger_at p1 n2)
    (passenger_at p2 n2)
    (passenger_at p3 n4)

    ; Define how fast each of the passengers move (in seconds)
    (= (person_speed p1) 2)
    (= (person_speed p2) 3)
    (= (person_speed p3) 2)

    ; Define the speed of the elevators (in meters / second)
    (= (elevator_speed e1) 2)
    (= (elevator_speed e2) 3)

    ; Define the distance between the floors (in meters)
    (= (floor_distance n1 n2) 3)
    (= (floor_distance n2 n3) 4)
    (= (floor_distance n3 n4) 4)
    (= (floor_distance n4 n5) 3)
    (= (floor_distance n5 n4) 3)
    (= (floor_distance n4 n3) 4)
    (= (floor_distance n3 n2) 4)
    (= (floor_distance n2 n1) 3)
  )

  (:goal (and
    (passenger_at p1 n1)
    (passenger_at p2 n1)
    (passenger_at p3 n1)
  ))

  (:metric
    minimize (total-time)
  )
)
