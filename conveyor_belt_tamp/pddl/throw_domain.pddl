;Header and description

(define (domain throw_test)

;remove requirements that are not needed
(:requirements :strips)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (thrown-center)
    (thrown-right)
    (thrown-left)
)


; (:functions ;todo: define numeric functions here
; )

(:action throw-center
    :parameters ()
    :precondition (and )
    :effect (and 
        (thrown-center)
    )
)

(:action throw-right
    :parameters ()
    :precondition (and 
        (thrown-center)
    )
    :effect (and 
        (thrown-right)
    )
)

(:action throw-left
    :parameters ()
    :precondition (and 
        (thrown-right)
    )
    :effect (and 
        (thrown-left)
    )
)

;define actions here

)