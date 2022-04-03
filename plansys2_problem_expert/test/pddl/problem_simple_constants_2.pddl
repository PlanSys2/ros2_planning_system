( define ( problem problem_1 )
( :domain plansys2 )
( :objects
	jack john - person
	m1 - message
	leia lema - robot
	kitchen bedroom - room
)
( :init
	( robot_at leia kitchen )
	( person_at jack bedroom )
)
( :goal
	( and
		( robot_talk leia m1 jack )
	)
)
)
