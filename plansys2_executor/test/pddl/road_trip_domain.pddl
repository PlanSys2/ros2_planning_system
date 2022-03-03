(define (domain road_trip)

  ;; Specify the features of PDDL used by the domain.
  (:requirements :adl :durative-actions :fluents :negative-preconditions :strips :typing)

  ;; Specify the domain's parameter types.
  (:types car waypoint - object)

  ;; Declare the predicates.
  ;; connected - ?wp1 (waypoint) is connected to ?wp2 (waypoint)
  ;; fuel_at   - ?wp (waypoint) has fuel
  ;; car_at    - ?s (car) is at ?wp (waypoint)
  ;; visited   - ?wp (waypoint) has been visited
  (:predicates
    (connected ?wp1 ?wp2 - waypoint)
    (fuel_at ?wp - waypoint)
    (car_at ?s - car ?wp - waypoint)
    (visited ?wp - waypoint)
  )

  ;; Declare the static functions.
  ;; distance(?wp1, ?wp2) - distance between ?wp1 (waypoint) and ?wp2 (waypoint), m
  ;; fuel_level(?s)       - fuel level of ?s (car), percent
  ;; min_fuel_level(?s)   - minimum fuel level of ?s (car) to maintain, percent
  ;; range(?s)            - range of ?s (car) given a full tank, m
  ;; fuel_rate(?s)        - rate fuel tank fills for ?s (car), percent/s
  ;; speed(?s)            - speed of ?s (car), m/s
  (:functions
    (distance ?wp1 ?wp2 - waypoint)
    (fuel_level ?s - car)
    (min_fuel_level ?s - car)
    (range ?s - car)
    (fuel_rate ?s - car)
    (speed ?s - car)
  )

  ;; Declare durative actions.

  ;; Move ?s (car) from ?wp1 (waypoint) to ?wp2 (waypoint).
  ;; duration = distance(?wp1, ?wp2) / speed(?s)
  ;; condition - at start -> ?s (car) is at ?wp1 (waypoint)
  ;;             at start -> ?wp1 (waypoint) is connected to ?wp2 (waypoint)
  ;;             at start -> fuel_level(?s) > min_fuel_level(?s) + distance(?wp1, ?wp2) / range(?s)
  ;; effect    - at start -> ?s (car) is NOT at ?wp1 (waypoint)
  ;;             at end -> ?s (car) is at ?wp2 (waypoint)
  ;;             at end -> fuel_level(?s) -= distance(?wp1, ?wp2) / range(?s)
  (:durative-action move
    :parameters (?s - car ?wp1 ?wp2 - waypoint)
    :duration (= ?duration (/ (distance ?wp1 ?wp2) (speed ?s)))
    :condition (and (at start (car_at ?s ?wp1))
                    (at start (connected ?wp1 ?wp2))
                    (at start (> (fuel_level ?s)
                                 (+ (min_fuel_level ?s)
                                    (/ (distance ?wp1 ?wp2) (range ?s))))))
    :effect (and (at start (not (car_at ?s ?wp1)))
                 (at end (car_at ?s ?wp2))
                 (at end (decrease (fuel_level ?s)
                                   (/ (distance ?wp1 ?wp2) (range ?s)))))
  )

  ;; Visit ?wp (waypoint) with ?s (car).
  ;; This is largely a placeholder for a possible future action with real duration and more conditions/effects.
  ;; duration = 1.0
  ;; condition - over all -> ?s (car) is at ?wp (waypoint)
  ;; effect    - at end -> ?wp (waypoint) has been visited
  (:durative-action visit
    :parameters (?s - car ?wp - waypoint)
    :duration (= ?duration 1.0)
    :condition (over all (car_at ?s ?wp))
    :effect (at end (visited ?wp))
  )

  ;; Refuel ?s (car) at ?wp (waypoint).
  ;; duration = ((1.0 - fuel_level) / fuel_rate rate)
  ;; condition - at start -> fuel_level(?s) <= 1.0
  ;;             over all -> ?s (car) is at ?wp (waypoint)
  ;;             over all -> ?wp (waypoint) has fuel
  ;; effect    - at end -> fuel_level(?s) = 1.0
  (:durative-action refuel
    :parameters (?s - car ?wp - waypoint)
    :duration (= ?duration (/ (- 1.0 (fuel_level ?s)) (fuel_rate ?s)))
    :condition (and (at start (<= (fuel_level ?s) 1.0))
                    (over all (car_at ?s ?wp))
                    (over all (fuel_at ?wp)))
    :effect (and (at end (assign (fuel_level ?s) 1.0)))
  )
)
