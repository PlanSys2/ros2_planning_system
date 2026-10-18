(define (problem object_positions_p1)
  (:domain object_positions)
  (:objects
    kobuki - robot
    dock hall - room
    crate - box
  )
  (:init
    (connected dock hall)
    (connected hall dock)
    (= (pos kobuki) dock)
    (= (box_pos crate) dock)
  )
  (:goal (and
    (= (box_pos crate) hall)
  ))
)
