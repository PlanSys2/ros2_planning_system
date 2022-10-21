(define (problem three-block)
(:domain blocksworld)
(:objects b1 b2 b3 - block)
(:init
(ontable b1)
(clear b1)
(unknown (ontable b3))
(unknown (clear b3))
(unknown (on b3 b2))
(unknown (ontable b2))
(unknown (clear b2))
(unknown (on b2 b3))
(or
(not (on b3 b2))
(not (on b2 b3))
)
(or
(not (on b2 b3))
(not (on b3 b2))
)
(oneof
(clear b3)
(clear b2)
)
(oneof
(ontable b3)
(ontable b2)
)
(oneof
(ontable b3)
(on b3 b2)
)
(oneof
(ontable b2)
(on b2 b3)
)
(oneof
(clear b3)
(on b2 b3)
)
(oneof
(clear b2)
(on b3 b2)
)
)
(:goal
(and
(on b2 b1)
(on b3 b2))
)
)

