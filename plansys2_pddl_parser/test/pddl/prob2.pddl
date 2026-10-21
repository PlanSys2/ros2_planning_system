(define (problem open-the-door)
  (:domain simple-door)
  (:init
    (not_door_open)
  )
  (:goal
    (and (door_open))
  )
)
