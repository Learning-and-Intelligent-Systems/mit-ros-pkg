(define (problem scrape)
	(:domain scrape-ingreds)
	(:objects mb
              cs
			  larm
			  rarm)
	(:init (MIXING-BOWL mb)
		   (GRIPPER larm)
		   (GRIPPER rarm)
		   (SPATULA-GRIPPER rarm)
           (COOKIE-SHEET cs)
		   (reset larm)
		   (reset rarm)
		   (free larm)
		   (not (poured-out mb))
		   (not (scraped mb))
		   (not (in-mb rarm))
		   (not (over-cs larm))
		   (not (over-cs rarm))
		   (not (cs-zone cs)))
	(:goal (and (scraped mb) (reset larm) (reset rarm)))
)

	
