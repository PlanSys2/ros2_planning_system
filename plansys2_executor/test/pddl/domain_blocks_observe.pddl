(define (domain blocksworld)
(:requirements :strips :typing :disjunctive-preconditions)
(:types block)
(:predicates (clear ?x - block)
             (ontable ?x - block)
             (on ?x ?y - block)
)

  (:action senseON
   :parameters (?b1 ?b2 - block)
   :observe (on ?b1 ?b2))

  (:action senseCLEAR
   :parameters (?b1 - block)
   :observe (clear ?b1))

  (:action senseONTABLE
   :parameters (?b1 - block)
   :observe (ontable ?b1))

(:action move-b-to-b
  :parameters (?bm ?bf ?bt - block)
  :precondition (and (clear ?bm) (clear ?bt) (on ?bm ?bf))
  :effect (and (not (clear ?bt)) (not (on ?bm ?bf))
               (on ?bm ?bt) (clear ?bf)))

(:action move-to-t
  :parameters (?b ?bf - block)
  :precondition (and (clear ?b) (on ?b ?bf))
  :effect (and (ontable ?b) (not (on ?b ?bf)) (clear ?bf))
  )

(:action move-t-to-b
  :parameters (?bm ?bt - block)
  :precondition (and (clear ?bm) (clear ?bt) (ontable ?bm))
  :effect (and (not (clear ?bt)) (not (ontable ?bm))
               (on ?bm ?bt))))
