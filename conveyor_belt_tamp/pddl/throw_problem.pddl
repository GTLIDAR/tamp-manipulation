(define (problem throw_problem) (:domain throw_test)
; (:objects 
; )

(:init
    ;todo: put the initial state's facts and numeric values here
)

(:goal (and
    ;todo: put the goal condition here
    (thrown-center)
    (thrown-right)
    (thrown-left)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
