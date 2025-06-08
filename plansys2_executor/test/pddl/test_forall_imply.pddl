(define (domain test_forall_imply)
  (:requirements :strips :adl :typing :durative-actions :fluents)
  (:types
    object
    item - object
    location - object
  )
  (:predicates
    (item_at ?item - item ?location - location)
    (item_checked ?item - item)
    (all_items_checked ?location - location)
  )
  (:durative-action check_items
    :parameters (?location - location)
    :duration (= ?duration 1)
    :condition
      (and (over all 
        (forall (?item - item)
          (imply (item_at ?item ?location) (item_checked ?item))
        )
      ))
    :effect
      (and
        (at end (all_items_checked ?location))
      )
  )
)