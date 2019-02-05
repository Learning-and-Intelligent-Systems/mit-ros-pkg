(define (problem put-cs-in-oven)
	(:domain ovening)
	(:objects cs
			  robot
			  oven)
	(:init (ROBOT robot)
		   (OVEN oven)
		   (COOKIE-SHEET cs)
		   (at-table robot)
		   (not (at-oven robot))
           (not (aligned robot oven))
           (not (carry robot cs))
           (not (open oven))
           (not (in-oven cs))
           (not (cooked cs)))
	(:goal (and (cooked cs) (open oven)))
)

	
