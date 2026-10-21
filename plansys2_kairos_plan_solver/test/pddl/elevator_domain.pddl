(define (domain temporal-elevators)

  ;; Specify the features of PDDL that the domain uses.
  (:requirements :typing :fluents :durative-actions :negative-preconditions)

  ;; Specify the domain's parameter types.
  (:types elevator passenger num - object)

  ;; Specify the domains's state variables.
  ;; passenger_at - Is ?person (passenger) at ?floor (num)?
  ;; boarded      - Has ?person (passenger) boarded ?lift (elevator)?
  ;; lift_at      - Is ?lift (elevator) at ?floor (num)?
  ;; next         - Is ?n2 (num) next after ?n1 (num)?
  (:predicates
    (passenger_at ?person - passenger ?floor - num)
    (boarded ?person - passenger ?lift - elevator)
    (lift_at ?lift - elevator ?floor - num)
    (next ?n1 - num ?n2 - num)
  )

  ;; Declare static functions that can be used in action expressions.
  ;; person_speed(?person)    - the speed of ?person (passenger)
  ;; elevator_speed(?lift)    - the speed of ?lift (elevator)
  ;; floor_distance(?f1, ?f2) - the distance between ?f1 (num) and ?f2 (num)
  (:functions
    (person_speed ?person - passenger)
    (elevator_speed ?lift - elevator)
    (floor_distance ?f1 ?f2 - num)
  )

  ;; Move ?lift (elevator) from ?cur (num) UP to ?nxt (num).
  ;; duration = floor_distance(?cur, ?nxt) / elevator_speed(?lift)
  ;; condition - at start -> ?lift (elevator) is at ?cur (num)
  ;;             over all -> ?nxt (num) is next after ?cur (num)
  ;; efect - at start -> ?lift (elevator) is NOT at ?cur (num)
  ;;         at end   -> ?lift (elevator is at ?nxt (num)
  (:durative-action move-up
    :parameters (?lift - elevator ?cur ?nxt - num)
    :duration (= ?duration (/ (floor_distance ?cur ?nxt)
                              (elevator_speed ?lift)))
    :condition (and (at start (lift_at ?lift ?cur))
                    (over all (next ?cur ?nxt)))
    :effect (and (at start (not (lift_at ?lift ?cur)))
                 (at end (lift_at ?lift ?nxt)))
  )

  ;; Move ?lift (elevator) from ?cur (num) DOWN to ?nxt (num).
  ;; duration = floor_distance(?cur, ?nxt) / elevator_speed(?lift)
  ;; condition - at start -> ?lift (elevator) is at ?cur (num)
  ;;             over all -> ?cur (num) is next after ?nxt (num)
  ;; efect - at start -> ?lift (elevator) is NOT at ?cur (num)
  ;;         at end   -> ?lift (elevator is at ?nxt (num)
  (:durative-action move-down
    :parameters (?lift - elevator ?cur ?nxt - num)
    :duration (= ?duration (/ (floor_distance ?cur ?nxt)
                              (elevator_speed ?lift)))
    :condition (and (at start (lift_at ?lift ?cur))
                    (over all (next ?nxt ?cur)))
    :effect (and (at start (not (lift_at ?lift ?cur)))
                 (at end (lift_at ?lift ?nxt)))
  )

  ;; Board ?per (passenger) onto ?lift (elevator) at ?flr (num).
  ;; duration = person_speed(?per)
  ;; condition - over all -> ?lift (elevator) is at ?flr (num)
  ;;             at start -> ?per (passenger) is at ?flr (num)
  ;; effect - at start -> ?per (passenger) is NOT at ?flr (num)
  ;;          at end   -> ?per (passenger) has boarded ?lift (elevator)
  (:durative-action board
    :parameters (?per - passenger ?flr - num ?lift - elevator)
    :duration (= ?duration (person_speed ?per))
    :condition (and (over all (lift_at ?lift ?flr))
                    (at start (passenger_at ?per ?flr)))
    :effect (and (at start (not (passenger_at ?per ?flr)))
                 (at end (boarded ?per ?lift)))
  )

  ;; Exit ?per (passenger) from ?lift (elevator) at ?flr (num).
  ;; duration = person_speed(?per)
  ;; condition - over all -> ?lift (elevator) is at ?flr (num)
  ;;             at start -> ?per (passenger) is boarded on ?lift (elevator)
  ;; effect - at end   -> ?per (passenger) is at ?flr (num)
  ;;          at start -> ?per (passenger) is NOT boarded on ?lift (elevator)
  (:durative-action leave
    :parameters (?per - passenger ?flr - num ?lift - elevator)
    :duration (= ?duration (person_speed ?per))
    :condition (and (over all (lift_at ?lift ?flr))
                    (at start (boarded ?per ?lift)))
    :effect (and (at end (passenger_at ?per ?flr))
                 (at start (not (boarded ?per ?lift))))
  )
)
