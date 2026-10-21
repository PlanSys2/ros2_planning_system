( define ( domain start_gives_over_all )
( :requirements :strips :typing :durative-actions )
( :types
	robot - object
	zone - object
)
( :predicates
	( robot_available ?robot0 - robot )
	( battery_full ?robot0 - robot )
	( robot_at ?robot0 - robot ?zone1 - zone )
	( moving ?robot0 - robot )
	( door_open ?zone0 - zone )
)
; The at start effect gives the over all requirement (moving ?robot0)
( :durative-action move
  :parameters ( ?robot0 - robot ?zone1 - zone ?zone2 - zone )
  :duration ( = ?duration 5 )
  :condition
	( and
		( at start ( robot_available ?robot0 ) )
		( at start ( robot_at ?robot0 ?zone1 ) )
		( at start ( door_open ?zone1 ) )
		( over all ( battery_full ?robot0 ) )
		( over all ( moving ?robot0 ) )
	)
  :effect
	( and
		( at start ( not ( robot_at ?robot0 ?zone1 ) ) )
		( at start ( not ( robot_available ?robot0 ) ) )
		( at start ( moving ?robot0 ) )
		( at end ( not ( moving ?robot0 ) ) )
		( at end ( robot_at ?robot0 ?zone2 ) )
		( at end ( robot_available ?robot0 ) )
	)
)
)
