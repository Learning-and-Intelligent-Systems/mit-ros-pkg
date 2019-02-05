(define (domain ovening)
    (:predicates (COOKIE-SHEET ?x)
                 (ROBOT ?x)
                 (OVEN ?x)
                 (aligned ?x ?y)
                 (at-oven ?x)
                 (at-table ?x)
                 (open ?x)
                 (in-oven ?x)
                 (carry ?x ?y)
                 (cooked ?x))
    (:action drive-to-oven
        :parameters (?x)
        :precondition (ROBOT ?x)
        :effect (and (at-oven ?x)
                     (not (at-table ?x))))
    (:action align
        :parameters (?x ?y)
        :precondition (and (ROBOT ?x) (OVEN ?y)
                           (at-oven ?x))
        :effect (and (aligned ?x ?y)))
    (:action open-oven
        :parameters (?x ?y ?z)
        :precondition (and (ROBOT ?x) (OVEN ?y) (COOKIE-SHEET ?z)
                           (aligned ?x ?y)
                           (not (carry ?x ?z)))
        :effect (and (open ?y)))
    (:action drive-to-table
        :parameters (?x ?y)
        :precondition (and (ROBOT ?x) (OVEN ?y))
        :effect (and (at-table ?x)
                     (not (at-oven ?x))
                     (not (aligned ?x ?y))))
    (:action grab
        :parameters (?x ?y)
        :precondition (and (ROBOT ?x) (COOKIE-SHEET ?y)
                           (at-table ?x))
        :effect (and (carry ?x ?y)))
    (:action put-in-oven
        :parameters (?x ?y ?z)
        :precondition (and (ROBOT ?x) (OVEN ?y) (COOKIE-SHEET ?z)
                           (at-oven ?x)
                           (open ?y)
                           (carry ?x ?z))
        :effect (and (not (carry ?x ?z)) (in-oven ?z)))
    (:action close-oven
        :parameters (?x ?y)
        :precondition (and (ROBOT ?x) (OVEN ?y)
                           (at-oven ?x)
                           (open ?y))
        :effect (and (not (open ?y))))
    (:action wait
        :parameters (?x ?y ?z)
        :precondition (and (ROBOT ?x) (OVEN ?y) (COOKIE-SHEET ?z)
                           (not (open ?y))
                           (in-oven ?z)
                           (at-oven ?x))
        :effect (and (cooked ?z)))
)
