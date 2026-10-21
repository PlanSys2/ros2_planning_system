( define ( domain move_at_end )
( :requirements :strips :typing :durative-actions )
( :types
	robot - object
	zone - object
)
( :predicates
	( robot_available ?robot0 - robot )
	( battery_full ?robot0 - robot )
	( robot_at ?robot0 - robot ?zone1 - zone )
)
; Nothing changes at the start: the action may still be in a plan made while it runs
( :durative-action move
  :parameters ( ?robot0 - robot ?zone1 - zone ?zone2 - zone )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_at ?robot0 ?zone1 ) )
		( over all ( battery_full ?robot0 ) )
	)
  :effect
	( and
		( at end ( not ( robot_at ?robot0 ?zone1 ) ) )
		( at end ( robot_at ?robot0 ?zone2 ) )
	)
)
)
