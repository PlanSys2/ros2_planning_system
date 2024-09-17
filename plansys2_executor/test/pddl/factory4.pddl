( define (domain factory)
( :requirements :strips :adl :typing :fluents )
( :types
	robot - object
	zone - object
	piece - object
	car - object
)
( :predicates
	( robot_available ?robot0 - robot )
  ( battery_full ?robot0 - robot )
	( robot_at ?robot0 - robot ?zone1 - zone )
	( piece_at ?piece0 - piece ?zone1 - zone )
	( piece_is_wheel ?piece0 - piece )
	( piece_is_body_car ?piece0 - piece )
	( piece_is_steering_wheel ?piece0 - piece )
	( piece_not_used ?piece0 - piece )
	( is_assembly_zone ?zone0 - zone )
	( car_assembled ?car0 - car )
)
( :action move
  :parameters (?robot0 - robot ?zone1 - zone ?zone2 - zone)
  :precondition	(and
		(robot_available ?robot0)
		(robot_at ?robot0 ?zone1)
    (battery_full ?robot0)
	)
  :effect	(and
		(not (robot_at ?robot0 ?zone1))
		(robot_at ?robot0 ?zone2)
		(robot_available ?robot0)
	)
)
( :action transport
  :parameters (?robot0 - robot ?piece1 - piece ?zone2 - zone ?zone3 - zone)
  :precondition	(and
		(robot_available ?robot0)
		(robot_at ?robot0 ?zone2)
		(piece_at ?piece1 ?zone2)
	)
  :effect (and
		(not (robot_at ?robot0 ?zone2))
		(not (piece_at ?piece1 ?zone2))
		(robot_at ?robot0 ?zone3)
		(piece_at ?piece1 ?zone3)
		(robot_available ?robot0)
	)
)
( :action assemble
  :parameters (?robot0 - robot ?zone1 - zone ?piece2 - piece ?piece3 - piece ?piece4 - piece ?car5 - car)
  :precondition (and
		(robot_available ?robot0)
		(is_assembly_zone ?zone1)
		(robot_at ?robot0 ?zone1)
		(piece_at ?piece2 ?zone1)
		(piece_at ?piece3 ?zone1)
		(piece_at ?piece4 ?zone1)
		(piece_not_used ?piece2)
		(piece_not_used ?piece3)
		(piece_not_used ?piece4)
		(piece_is_wheel ?piece2)
		(piece_is_body_car ?piece3)
		(piece_is_steering_wheel ?piece4)
	)
  :effect (and
		(not (piece_not_used ?piece2))
		(not (piece_not_used ?piece3))
		(not (piece_not_used ?piece4))
		(car_assembled ?car5)
		(robot_available ?robot0)
	)
)
)
