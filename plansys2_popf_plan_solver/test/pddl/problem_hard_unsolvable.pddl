; Unsolvable problem that popf cannot discard quickly: robot0 must be in two
; rooms at once, which only the delete relaxation allows, so the search explores
; the whole state space. Used to check that solver timeouts are enforced.
(define (problem hard_unsolvable)
(:domain simple)
(:objects
  robot0 robot1 robot2 robot3 robot4 robot5 robot6 robot7 - robot
  room0 room1 room2 room3 room4 room5 room6 room7 room8 room9 room10 room11 room12 room13 room14 room15 room16 room17 room18 room19 room20 room21 room22 room23 room24 room25 room26 room27 room28 room29 room30 room31 room32 room33 room34 room35 room36 room37 room38 room39 - room
)
(:init
  (robot_at robot0 room0)
  (robot_at robot1 room0)
  (robot_at robot2 room0)
  (robot_at robot3 room0)
  (robot_at robot4 room0)
  (robot_at robot5 room0)
  (robot_at robot6 room0)
  (robot_at robot7 room0)
)
(:goal
  (and
    (robot_at robot0 room39)
    (robot_at robot1 room36)
    (robot_at robot2 room33)
    (robot_at robot3 room30)
    (robot_at robot4 room27)
    (robot_at robot5 room24)
    (robot_at robot6 room21)
    (robot_at robot7 room18)
    (robot_at robot0 room20)
  )
)
)
