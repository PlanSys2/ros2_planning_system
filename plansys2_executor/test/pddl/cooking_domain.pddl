(define (domain cooking)
(:requirements :strips :typing :adl :fluents :durative-actions :typing)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
ingredient
dish
zone 
robot
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(battery_full ?r - robot)
(robot_at ?r - robot ?z - zone)
(ingredient_at ?i - ingredient ?z - zone)

(is_egg ?i - ingredient)
(is_oil ?i - ingredient)
(is_salt ?i - ingredient)
(is_flour ?i - ingredient)
(is_sugar ?i - ingredient)
(is_pasta ?i - ingredient)
(is_water ?i - ingredient) 

(is_omelette ?d - dish) 
(is_cake ?d - dish) 
(is_spaghetti ?d - dish) 

(is_cooking_zone ?z - zone)
(is_pantry_zone ?z - zone)
(is_fridge_zone ?z - zone)
(is_watertap_zone ?z - zone)
(is_recharge_zone ?z - zone)

(dish_prepared ?d - dish)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1)))
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
    )
)

(:durative-action transport
    :parameters (?r - robot ?i - ingredient ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all(battery_full ?r))
        (at start(robot_at ?r ?z1))
        (at start(ingredient_at ?i ?z1))
    )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(ingredient_at ?i ?z1)))
        (at end(ingredient_at ?i ?z2))
    )
)

(:durative-action recharge
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_recharge_zone ?z))
        (over all(robot_at ?r ?z))
      )
    :effect (and
        (at end(battery_full ?r))
    )
)

(:durative-action cook_omelette
    :parameters (?r - robot ?z - zone ?i1 ?i2 ?i3 - ingredient ?d - dish)
    :duration ( = ?duration 5)
    :condition (and
        (over all(battery_full ?r))

        (at start(is_cooking_zone ?z))
        (over all(robot_at ?r ?z))

        (at start(is_egg ?i1))
        (at start(is_oil ?i2))
        (at start(is_salt ?i3))
        (at start(ingredient_at ?i1 ?z))
        (at start(ingredient_at ?i2 ?z))
        (at start(ingredient_at ?i3 ?z))
        (at start(is_omelette ?d))
    )
    :effect (and
        (at end(dish_prepared ?d))

    )
)

(:durative-action cook_cake
    :parameters (?r - robot ?z - zone ?i1 ?i2 ?i3 - ingredient ?d - dish)
    :duration ( = ?duration 5)
    :condition (and
        (over all(battery_full ?r))

        (at start(is_cooking_zone ?z))
        (over all(robot_at ?r ?z))

        (at start(is_flour ?i1))
        (at start(is_egg ?i2))
        (at start(is_sugar ?i3))
        (at start(ingredient_at ?i1 ?z))
        (at start(ingredient_at ?i2 ?z))
        (at start(ingredient_at ?i3 ?z))
        (at start(is_cake ?d))
    )
    :effect (and
        (at end(dish_prepared ?d))

    )
)

(:durative-action cook_spaghetti
    :parameters (?r - robot ?z - zone ?i1 ?i2 ?i3 ?i4 - ingredient ?d - dish)
    :duration ( = ?duration 5)
    :condition (and
        (over all(battery_full ?r))

        (at start(is_cooking_zone ?z))
        (over all(robot_at ?r ?z))

        (at start(is_water ?i1))
        (at start(is_pasta ?i2))
        (at start(is_oil ?i3))
        (at start(is_salt ?i4))
        (at start(ingredient_at ?i1 ?z))
        (at start(ingredient_at ?i2 ?z))
        (at start(ingredient_at ?i3 ?z))
        (at start(ingredient_at ?i4 ?z))
        (at start(is_spaghetti ?d))
    )
    :effect (and
        (at end(dish_prepared ?d))

    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
