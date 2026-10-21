(define (domain powered_doors)
  (:requirements :strips :typing :conditional-effects)
  (:types
    robot room door button panel
  )
  (:predicates
    (robot_at ?r - robot ?rm - room)
    (door_between ?d - door ?from ?to - room)
    (door_open ?d - door)
    (button_in ?b - button ?rm - room)
    (controls ?b - button ?d - door)
    (panel_in ?p - panel ?rm - room)
    (powered ?b - button)
    (feeds ?p - panel ?b - button)
  )
  (:action move
    :parameters (?r - robot ?d - door ?from ?to - room)
    :precondition (and
      (robot_at ?r ?from)
      (door_between ?d ?from ?to)
      (door_open ?d)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )
  (:action restore_power
    :parameters (?r - robot ?p - panel ?b - button ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (panel_in ?p ?rm)
      (feeds ?p ?b)
    )
    :effect (and
      (powered ?b)
    )
  )
  (:action press_button
    :parameters (?r - robot ?b - button ?d - door ?rm - room)
    :precondition (and
      (robot_at ?r ?rm)
      (button_in ?b ?rm)
      (controls ?b ?d)
    )
    :effect (and
      (when (powered ?b) (door_open ?d))
    )
  )
)
