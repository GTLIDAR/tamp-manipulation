(define (problem 1obj_problem) (:domain conveyor_belt_domain_coupled)
(:objects
    iiwa - robot
    belt - belt


    bin_0 - bin
    bin_1 - bin
    bin_2 - bin
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (free iiwa)
    (ready-to-move iiwa)

    (todo box_0)

)

(:goal (and
    ;todo: put the goal condition here
    (in box_0 bin_0)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
