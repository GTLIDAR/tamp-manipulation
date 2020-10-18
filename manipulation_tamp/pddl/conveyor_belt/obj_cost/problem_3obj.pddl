(define (problem 4obj_problem) (:domain conveyor_belt_4obj)
(:objects
    iiwa - robot

    bin_0 - normal_bin
    bin_1 - normal_bin
    bin_2 - normal_bin
    bin_3 - far_bin

    belt - conveyor_belt
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (free iiwa)
    (ready-to-move iiwa)

    (on box_0 belt)
    (on box_1 belt)
    (on box_2 belt)

    (unobstructed box_0 box_0)
    ; (unobstructed box_0 box_1)
    ; (unobstructed box_0 box_2)


    (unobstructed box_1 box_0)
    (unobstructed box_1 box_1)
    (unobstructed box_1 box_2)

    (unobstructed box_2 box_0)
    (unobstructed box_2 box_1)
    (unobstructed box_2 box_2)

)

(:goal (and
    ;todo: put the goal condition here
    (in box_0 bin_1)
    (in box_1 bin_0)
    (in box_2 bin_0)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
