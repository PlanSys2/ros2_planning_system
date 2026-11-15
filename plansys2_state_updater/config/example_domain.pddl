(define (domain state_updater_example)
  (:requirements :typing :fluents :object-fluents :durative-actions :negative-preconditions
    :continuous-effects)
  (:types sensor waypoint mode contact)
  (:predicates
    (leak_detected)
    (low_battery)
    (sensor_stale ?s - sensor)
    (sensor_ok ?s - sensor)
    (reachable ?w - waypoint)
    (mode_is ?m - mode)
    (contact_at ?c - contact ?w - waypoint)
  )
  (:functions
    ; A type applies to all the functions before it: (energy) is a number
    (energy) - number
    (location) - waypoint
  )
  (:durative-action inspect
    :parameters ()
    :duration (= ?duration 10)
    :condition (and (over all (not (leak_detected))) (at start (>= (energy) 20)))
    :effect (and (decrease (energy) (* #t 0.5)))
  )
)
