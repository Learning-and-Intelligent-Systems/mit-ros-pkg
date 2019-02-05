(define (problem mix-1-ingred)
	(:domain mix-ingreds)
	(:objects mb
			  larm
			  rarm)
	(:init (MIXING-BOWL mb)
		   (GRIPPER larm)
		   (GRIPPER rarm)
		   (SPATULA-GRIPPER rarm)
		   (reset larm)
		   (reset rarm)
		   (free larm)
		   (not (mixed mb))
		   (not (in-mb rarm))
		   (not (over-mb larm))
		   (not (over-mb rarm))
		   (not (mix-zone mb)))
	(:goal (and (mixed mb) (reset larm) (reset rarm)))
)

	
