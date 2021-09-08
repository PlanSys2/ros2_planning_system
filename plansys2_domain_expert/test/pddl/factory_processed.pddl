(define (domain factory)
(:requirements :adl :durative-actions :fluents :strips :typing )

(:types
	robot - object
	zone - object
	piece - object
	car - object
)

(:constants
)

(:predicates
	( car_assembled ?car0 - car )
	( is_assembly_zone ?zone0 - zone )
	( piece_at ?piece0 - piece ?zone1 - zone )
	( piece_is_body_car ?piece0 - piece )
	( piece_is_steering_wheel ?piece0 - piece )
	( piece_is_wheel ?piece0 - piece )
	( piece_not_used ?piece0 - piece )
	( robot_at ?robot0 - robot ?zone1 - zone )
	( robot_available ?robot0 - robot )
)

(:functions
)

(:durative-action move
  :parameters ( ?robot0 - robot ?zone1 - zone ?zone2 - zone )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( robot_at ?robot0 ?zone1 ) )
	)
  :effect
	( and
		( at start ( not ( robot_at ?robot0 ?zone1 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at end ( robot_at ?robot0 ?zone2 ) )
		( at end ( robot_available ?robot0 ) )
	)
)
(:durative-action transport
  :parameters ( ?robot0 - robot ?piece1 - piece ?zone2 - zone ?zone3 - zone )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( robot_at ?robot0 ?zone2 ) )
		( at start ( piece_at ?piece1 ?zone2 ) )
	)
  :effect
	( and
		( at start ( not ( robot_at ?robot0 ?zone2 ) ) )
		( at start ( not ( piece_at ?piece1 ?zone2 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at end ( robot_at ?robot0 ?zone3 ) )
		( at end ( piece_at ?piece1 ?zone3 ) )
		( at end ( robot_available ?robot0 ) )
	)
)
(:durative-action assemble
  :parameters ( ?robot0 - robot ?zone1 - zone ?piece2 - piece ?piece3 - piece ?piece4 - piece ?car5 - car )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( is_assembly_zone ?zone1 ) )
		( at start ( robot_at ?robot0 ?zone1 ) )
		( at start ( piece_at ?piece2 ?zone1 ) )
		( at start ( piece_at ?piece3 ?zone1 ) )
		( at start ( piece_at ?piece4 ?zone1 ) )
		( at start ( piece_not_used ?piece2 ) )
		( at start ( piece_not_used ?piece3 ) )
		( at start ( piece_not_used ?piece4 ) )
		( at start ( piece_is_wheel ?piece2 ) )
		( at start ( piece_is_body_car ?piece3 ) )
		( at start ( piece_is_steering_wheel ?piece4 ) )
	)
  :effect
	( and
		( at start ( not ( piece_not_used ?piece2 ) ) )
		( at start ( not ( piece_not_used ?piece3 ) ) )
		( at start ( not ( piece_not_used ?piece4 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at end ( car_assembled ?car5 ) )
		( at end ( robot_available ?robot0 ) )
	)
)
)
