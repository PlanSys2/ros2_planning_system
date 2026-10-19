(define (problem object_positions_p3)
  (:domain object_positions)
  (:objects
    kobuki - robot
    dock hall office - room
    crate - box
  )
  (:init
    (connected dock hall)
    (connected hall dock)
    (connected office hall)
    (= (pos kobuki) dock)
    (= (box_pos crate) hall)
  )
  (:goal (and
    (= (box_pos crate) office)
  ))
)
