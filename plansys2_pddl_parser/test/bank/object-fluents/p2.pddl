(define (problem object_positions_p2)
  (:domain object_positions)
  (:objects
    kobuki - robot
    dock hall lab office storage - room
    crate barrel - box
  )
  (:init
    (connected dock hall)
    (connected hall dock)
    (connected hall lab)
    (connected lab hall)
    (connected lab office)
    (connected office lab)
    (connected hall storage)
    (connected storage hall)
    (= (pos kobuki) dock)
    (= (box_pos crate) lab)
    (= (box_pos barrel) storage)
  )
  (:goal (and
    (= (box_pos crate) office)
    (= (box_pos barrel) dock)
    (= (pos kobuki) hall)
  ))
)
