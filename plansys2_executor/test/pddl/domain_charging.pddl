(define (domain charging)

  ;; Specify the features of PDDL that the domain uses.
  (:requirements :typing :fluents :durative-actions :negative-preconditions)

  ;; Specify the domain's parameter types.
  (:types robot waypoint - object)

  ;; Specify the domains's predicates.
  ;; robot_at   - Is ?r (robot) at ?wp (waypoint)?
  ;; charger_at - Is there a charger at ?wp (waypoint)?
  ;; connected  - Is ?wp1 (waypoint) connected to ?wp2 (waypoint)?
  ;; patrolled  - Has ?wp (waypoint) been patrolled?
  (:predicates
    (robot_at ?r - robot ?wp - waypoint)
    (charger_at ?wp - waypoint)
    (connected ?wp1 ?wp2 - waypoint)
    (patrolled ?wp - waypoint)
  )

  ;; Declare static functions that can be used in action expressions.
  ;; speed(?r)            - speed of ?r (robot)
  ;; max_range(?r)        - range of ?r (robot) given a fully charged battery
  ;; state_of_charge(?r)  - available battery of ?r (robot) as a percentage
  ;; distance(?wp1, ?wp2) - distance between ?wp1 (waypoint) and ?wp2 (waypoint)
  (:functions
    (speed ?r - robot)
    (max_range ?r - robot)
    (state_of_charge ?r - robot)
    (distance ?wp1 ?wp2 - waypoint)
  )

  ;; Move ?r (robot) from ?wp1 (waypoint) to ?wp2 (waypoint).
  ;; duration = distance(?wp1, ?wp2) / speed(?r)
  ;; condition - at start -> ?wp1 (waypoint) is connected to ?wp2 (waypoint)
  ;;             at start -> ?r (robot) is at ?wp1 (waypoint)
  ;;             at start -> state_of_charge(?r) >= distance(?wp1, ?wp2) / max_range(?r)
  ;; efect - at start -> ?r (robot) is NOT at ?wp1 (waypoint)
  ;;         at start -> state_of_charge(?r) -= distance(?wp1, ?wp2) / max_range(?r)
  ;;         at end   -> ?r (robot) is at ?wp2 (waypoint)
  (:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration (= ?duration (/ (distance ?wp1 ?wp2)
                              (speed ?r)))
    :condition (and (at start (connected ?wp1 ?wp2))
                    (at start (robot_at ?r ?wp1))
                    (at start (>= (state_of_charge ?r)
                                  (* 100 (/ (distance ?wp1 ?wp2) (max_range ?r))))))
    :effect (and (at start (not (robot_at ?r ?wp1)))
                 (at start (decrease (state_of_charge ?r)
                                     (* 100 (/ (distance ?wp1 ?wp2) (max_range ?r)))))
                 (at end (robot_at ?r ?wp2)))
  )

  ;; Patrol ?wp (waypoint) with ?r (robot).
  ;; duration = 5
  ;; condition - at start -> ?r (robot) is at ?wp (waypoint)
  ;; efect - at end -> ?wp (waypoint) has been patrolled
  (:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration (= ?duration 5)
    :condition (and (over all (robot_at ?r ?wp)))
    :effect (and (at end (patrolled ?wp)))
  )

  ;; Charge ?r (robot) at ?wp (waypoint).
  ;; duration = 5
  ;; condition - at start -> ?r (robot) is at ?wp (waypoint)
  ;;             at start -> ?wp (waypoint) has a charger
  ;; efect - at end -> state_of_charge(?r) = 100
  (:durative-action charge
    :parameters (?r - robot ?wp - waypoint)
    :duration (= ?duration 5)
    :condition (and (at start (<= (state_of_charge ?r) 100))
                    (over all (robot_at ?r ?wp))
                    (over all (charger_at ?wp)))
    :effect (and (at end (assign (state_of_charge ?r) 100)))
  )
)
