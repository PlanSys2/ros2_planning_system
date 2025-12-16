(define (domain suave)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
    :derived-predicates
    :existential-preconditions
    :disjunctive-preconditions
  )


	(:types
		pipeline
		robot
		numerical-object
	)

	(:constants
		ERROR_string a_inspect_pipeline a_search_pipeline c_thruster_1 c_thruster_2 c_thruster_3 c_thruster_4 c_thruster_5 c_thruster_6 f_follow_pipeline f_generate_search_path f_maintain_motion false_boolean fd_all_thrusters fd_follow_pipeline fd_recover_thrusters fd_spiral_high fd_spiral_low fd_spiral_medium fd_unground obs_water_visibility performance qa_inspect_efficiency_high qa_motion_efficiency_degraded qa_motion_efficiency_normal qa_performance_zero qa_search_efficiency_high qa_search_efficiency_low qa_search_efficiency_medium qa_water_visibility_high qa_water_visibility_low qa_water_visibility_medium true_boolean water_visibility - object
	)

  (:predicates
    (pipeline_found ?p - pipeline)
    (pipeline_inspected ?p - pipeline)

    (robot_started ?r - robot)
    (robot_not_started ?r - robot)

		(Action ?x)
		(Component ?x)
		(Function ?x)
		(FunctionDesign ?x)
		(QAvalue ?x)
		(QualityAttributeType ?x)
		(c_status ?x ?y)
		(differentFrom ?x ?y)
		(f_active ?x ?y)
		(fdBetterUtility ?x ?y)
		(fd_realisability ?x ?y)
		(fd_utility ?x ?y)
		(functionGrounding ?x ?y)
		(hasQAestimation ?x ?y)
		(inferred-Action ?x)
		(inferred-C_status ?x ?y)
		(inferred-Component ?x)
		(inferred-DifferentFrom ?x ?y)
		(inferred-F_active ?x ?y)
		(inferred-FdBetterUtility ?x ?y)
		(inferred-Fd_realisability ?x ?y)
		(inferred-Fd_utility ?x ?y)
		(inferred-Function ?x)
		(inferred-FunctionDesign ?x)
		(inferred-FunctionGrounding ?x ?y)
		(inferred-HasQAestimation ?x ?y)
		(inferred-Inconsistent)
		(inferred-IsQAtype ?x ?y)
		(inferred-QAvalue ?x)
		(inferred-Qa_has_value ?x ?y)
		(inferred-QualityAttributeType ?x)
		(inferred-RequiresC ?x ?y)
		(inferred-RequiresF ?x ?y)
		(inferred-SameAs ?x ?y)
		(inferred-SolvesF ?x ?y)
		(isQAtype ?x ?y)
		(lessThan ?x ?y)
		(qa_has_value ?x ?y)
		(requiresC ?x ?y)
		(requiresF ?x ?y)
		(sameAs ?x ?y)
		(solvesF ?x ?y)
		(equalTo ?x ?y)
  )


(:derived (inferred-Action ?x) 
	(and
		(Action ?x)
	)
)


(:derived (inferred-Action ?x) 
	(exists (?y)
 		(and
			(inferred-RequiresF ?x ?y)
		)
	)
 )


(:derived (inferred-C_status ?x ?y) 
	(and
		(c_status ?x ?y)
	)
)


(:derived (inferred-Component ?x) 
	(and
		(Component ?x)
	)
)


(:derived (inferred-Component ?x) 
	(exists (?y)
 		(and
			(inferred-C_status ?x ?y)
		)
	)
 )


(:derived (inferred-Component ?y) 
	(exists (?x)
 		(and
			(inferred-RequiresC ?x ?y)
		)
	)
 )


(:derived (inferred-DifferentFrom ?x ?y) 
	(and
		(differentFrom ?x ?y)
	)
)


(:derived (inferred-F_active ?f ?true_boolean) 
	(and
		(= ?true_boolean true_boolean)
		(exists (?fd)
 			(and
				(inferred-Function ?f)
				(inferred-FunctionDesign ?fd)
				(not (= ?fd fd_unground))
				(inferred-SolvesF ?fd ?f)
				(inferred-FunctionGrounding ?f ?fd)
			)
		)
 	)
 )


(:derived (inferred-F_active ?x ?y) 
	(and
		(f_active ?x ?y)
	)
)


(:derived (inferred-FdBetterUtility ?fd_better ?fd) 
	(exists (?x_better ?f ?x)
 		(and
			(inferred-Fd_utility ?fd_better ?x_better)
			(inferred-Function ?f)
			(inferred-FunctionDesign ?fd)
			(inferred-FunctionDesign ?fd_better)
			(lessThan ?x ?x_better)
			(inferred-SolvesF ?fd ?f)
			(inferred-Fd_utility ?fd ?x)
			(inferred-SolvesF ?fd_better ?f)
		)
	)
 )


(:derived (inferred-FdBetterUtility ?x ?y) 
	(and
		(fdBetterUtility ?x ?y)
	)
)


(:derived (inferred-Fd_realisability ?fd ?false_boolean) 
	(and
		(= ?false_boolean false_boolean)
		(exists (?c)
 			(and
				(inferred-RequiresC ?fd ?c)
				(inferred-Component ?c)
				(inferred-C_status ?c ERROR_string)
			)
		)
 	)
 )


(:derived (inferred-Fd_realisability ?fd1 ?false_boolean) 
	(and
		(= ?false_boolean false_boolean)
		(exists (?eqa ?eqav ?mqa ?mqav)
 			(and
				(inferred-FunctionDesign ?fd1)
				(inferred-HasQAestimation ?fd1 ?eqa)
				(inferred-IsQAtype ?eqa water_visibility)
				(inferred-Qa_has_value ?eqa ?eqav)
				(inferred-QAvalue ?mqa)
				(= ?mqa obs_water_visibility)
				(inferred-Qa_has_value ?mqa ?mqav)
				(inferred-IsQAtype ?mqa water_visibility)
				(lessThan ?mqav ?eqav)
			)
		)
 	)
 )


(:derived (inferred-Fd_realisability ?x ?y) 
	(and
		(fd_realisability ?x ?y)
	)
)


(:derived (inferred-Fd_utility ?fd ?qav) 
	(exists (?f ?qa)
 		(and
			(inferred-Function f_follow_pipeline)
			(inferred-FunctionDesign ?fd)
			(inferred-SolvesF ?fd ?f)
			(inferred-HasQAestimation ?fd ?qa)
			(inferred-IsQAtype ?qa performance)
			(inferred-Qa_has_value ?qa ?qav)
		)
	)
 )


(:derived (inferred-Fd_utility ?fd ?qav) 
	(exists (?f ?qa)
 		(and
			(inferred-Function f_generate_search_path)
			(inferred-FunctionDesign ?fd)
			(inferred-SolvesF ?fd ?f)
			(inferred-HasQAestimation ?fd ?qa)
			(inferred-IsQAtype ?qa performance)
			(inferred-Qa_has_value ?qa ?qav)
		)
	)
 )


(:derived (inferred-Fd_utility ?fd ?qav) 
	(exists (?f ?qa)
 		(and
			(inferred-Function f_maintain_motion)
			(inferred-FunctionDesign ?fd)
			(inferred-SolvesF ?fd ?f)
			(inferred-HasQAestimation ?fd ?qa)
			(inferred-IsQAtype ?qa performance)
			(inferred-Qa_has_value ?qa ?qav)
		)
	)
 )


(:derived (inferred-Fd_utility ?x ?y) 
	(and
		(fd_utility ?x ?y)
	)
)


(:derived (inferred-Function ?x) 
	(and
		(Function ?x)
	)
)


(:derived (inferred-Function ?x) 
	(exists (?y)
 		(and
			(inferred-F_active ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?x) 
	(exists (?y)
 		(and
			(inferred-FunctionGrounding ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?y) 
	(exists (?x)
 		(and
			(inferred-RequiresF ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?y) 
	(exists (?x)
 		(and
			(inferred-SolvesF ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(and
		(FunctionDesign ?x)
	)
)


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-Fd_realisability ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-Fd_utility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-HasQAestimation ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-RequiresC ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-SolvesF ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?y) 
	(exists (?x)
 		(and
			(inferred-FdBetterUtility ?x ?y)
		)
	)
 )

(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-FdBetterUtility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?y) 
	(exists (?x)
 		(and
			(inferred-FunctionGrounding ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionGrounding ?x ?y) 
	(and
		(functionGrounding ?x ?y)
	)
)


(:derived (inferred-HasQAestimation ?x ?y) 
	(and
		(hasQAestimation ?x ?y)
	)
)


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-C_status ?x ?y)
			(inferred-C_status ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-F_active ?x ?y)
			(inferred-F_active ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-Fd_realisability ?x ?y)
			(inferred-Fd_realisability ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-Fd_utility ?x ?y)
			(inferred-Fd_utility ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-IsQAtype ?x ?y)
			(inferred-IsQAtype ?x ?z)
			(= ?y ?z)
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-Qa_has_value ?x ?y)
			(inferred-Qa_has_value ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-SolvesF ?x ?y)
			(inferred-SolvesF ?x ?z)
			(= ?y ?z)
		)
	)
 )


(:derived (inferred-IsQAtype ?x ?y) 
	(and
		(isQAtype ?x ?y)
	)
)


(:derived (inferred-QAvalue ?x) 
	(and
		(QAvalue ?x)
	)
)


(:derived (inferred-QAvalue ?x) 
	(exists (?y)
 		(and
			(inferred-IsQAtype ?x ?y)
		)
	)
 )


(:derived (inferred-QAvalue ?x) 
	(exists (?y)
 		(and
			(inferred-Qa_has_value ?x ?y)
		)
	)
 )


(:derived (inferred-QAvalue ?y) 
	(exists (?x)
 		(and
			(inferred-HasQAestimation ?x ?y)
		)
	)
 )


(:derived (inferred-Qa_has_value ?x ?y) 
	(and
		(qa_has_value ?x ?y)
	)
)


(:derived (inferred-QualityAttributeType ?x) 
	(and
		(QualityAttributeType ?x)
	)
)


(:derived (inferred-QualityAttributeType ?y) 
	(exists (?x)
 		(and
			(inferred-IsQAtype ?x ?y)
		)
	)
 )


(:derived (inferred-RequiresC ?x ?y) 
	(and
		(requiresC ?x ?y)
	)
)


(:derived (inferred-RequiresF ?x ?y) 
	(and
		(requiresF ?x ?y)
	)
)


(:derived (inferred-SameAs ?x ?y) 
	(and
		(sameAs ?x ?y)
	)
)


(:derived (inferred-SolvesF ?fd_unground ?f) 
	(and
		(= ?fd_unground fd_unground)
		(and
			(inferred-Function ?f)
			(inferred-FunctionDesign fd_unground)
		)
	)
 )


(:derived (inferred-SolvesF ?x ?y) 
	(and
		(solvesF ?x ?y)
	)
)

  (:action start_robot
    :parameters (?r - robot)
    :precondition (and
      (robot_not_started ?r)
    )
    :effect (and
      (not (robot_not_started ?r))	
      (robot_started ?r)
    )
  )

    (:action reconfigure1
    :parameters (?f ?fd_goal)
    :precondition (and
      (Function ?f)
      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))
      (not
        (exists (?fd)
          (and
            (inferred-SolvesF ?fd ?f)
            (FunctionDesign ?fd)
            (functionGrounding ?f ?fd)
          )
        )
      )
      (or 
      	(= ?fd_goal fd_unground)
        (not
          (exists (?fd)
            (and
              (inferred-SolvesF ?fd ?f)
              (not (inferred-Fd_realisability ?fd false_boolean))
              (inferred-FdBetterUtility  ?fd ?fd_goal)
            )
          )
        )
      )
    )
    :effect (and
      (functionGrounding ?f ?fd_goal)
    )
  )
  
  (:action reconfigure2
    :parameters (?f ?fd_initial ?fd_goal)
    :precondition (and
      (not (= ?fd_initial ?fd_goal))

      (Function ?f)
      (FunctionDesign ?fd_initial)
      (functionGrounding ?f ?fd_initial)

      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))

      (or (= ?fd_goal fd_unground)
        (not
          (exists (?fd)
            (and
              (inferred-SolvesF ?fd ?f)
              (not (inferred-Fd_realisability ?fd false_boolean))
              (inferred-FdBetterUtility  ?fd ?fd_goal)
            )
          )
        )
      )
    )
    :effect (and
      (not (functionGrounding ?f ?fd_initial))
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action search_pipeline
    :parameters (?p - pipeline ?r - robot)
    :precondition (and
      (robot_started ?r)
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (inferred-Action ?a)
          (= ?a a_search_pipeline)
          (not (= ?f1 ?f2))
          (inferred-RequiresF ?a ?f1)
          (inferred-RequiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
        )
      )
      (not (inferred-F_active f_follow_pipeline true_boolean))
    )
    :effect (and
      (pipeline_found ?p)
    )
  )

  (:action inspect_pipeline
    :parameters (?p - pipeline ?r - robot)
    :precondition (and
      (robot_started ?r)
      (pipeline_found ?p)
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (inferred-Action ?a)
          (= ?a a_inspect_pipeline)
          (not (= ?f1 ?f2))
          (inferred-RequiresF ?a ?f1)
          (inferred-RequiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
        )
      )
      (not (inferred-F_active f_generate_search_path true_boolean))
    )
    :effect (and
      (pipeline_inspected ?p)
    )
  )
)
