(define (domain kitchen)
(:requirements :strips :typing :adl :fluents :durative-actions)

(:types
ingredient - object
dish - object
chef - object
)

(:predicates
(chef_has ?c - chef ?i - ingredient)
(dish_ready ?d - dish)
(chef_cooking ?c - chef ?d - dish)
)

(:functions
    (prep_time ?d - dish)
)

(:durative-action cook
    :parameters (?c - chef ?d - dish)
    :duration ( = ?duration (prep_time ?d))
    :condition (and
        (at start(chef_cooking ?c ?d)))
    :effect (and
        (at end(dish_ready ?d))
    )
)

);; end Domain
