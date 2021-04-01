( define ( problem problem_1 )
( :domain test2 )
( :objects
	leia - robot
	room_1 room_2 - room
  corridor_1 - corridor
  zone_1_1 zone_1_2 zone_recharge - zone 
)
( :init
	( connected room_1 corridor_1 )
	( connected corridor_1 room_1 )
	( connected room_2 corridor_1 )
	( connected corridor_1 room_2 )
  ( connected room_1 zone_1_1 )
	( connected zone_1_1 room_1 )
  ( connected room_1 zone_1_2 )
	( connected zone_1_2 room_1 )
  ( connected room_2 zone_recharge )
	( connected zone_recharge room_2 )
	( charging_point_at zone_recharge )
	( battery_low leia )
	( robot_at leia zone_1_1 )
)
( :goal
	( and
		( robot_at leia zone_1_2 )
	)
)
)