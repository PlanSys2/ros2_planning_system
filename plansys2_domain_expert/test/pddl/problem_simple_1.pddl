(define (problem plansys2_1)
  (:domain plansys2)
  (:objects
    leia - robot
    Jack - person
    kitchen bedroom - room
    m1 - message
  )
  (:init
    (robot_at leia kitchen)
    (person_at Jack bedroom)


  )

  ;; The goal is to have both packages delivered to their destinations:
  (:goal (and
    (robot_talk leia m1 Jack) 
    )
  )
  )
