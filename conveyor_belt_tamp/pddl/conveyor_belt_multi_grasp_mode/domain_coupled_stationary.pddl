;Header and description

(define (domain conveyor_belt_domain_coupled)

;remove requirements that are not needed
(:requirements :strips :typing)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot - object
    belt - object
    box - object
    bin - object
)

; un-comment following line if constants are needed
(:constants
    box_0 - box
    box_1 - box
    box_2 - box
)

(:predicates ;todo: define predicates here
    (free ?r - robot)
    (ready-to-move ?r - robot)
    (holding ?r - robot ?b - box)
    (in ?b - box ?bin - bin)
    (on ?b - box ?belt - belt)

    (unblocked ?b1 - box ?b2 - box)

    (moved-to-object-top ?r - robot ?b - box)
    (moved-to-object-front ?r - robot ?b - box)
    (moved-to-bin ?r - robot ?b - box ?to - bin)

    (todo ?b - box)
)


; (:functions ;todo: define numeric functions here

; )

;define actions here
; (:action move-to-object-top
;     :parameters (?r - robot ?b - box)
;     :precondition (and
;         (free ?r)
;         (ready-to-move ?r)
;         (todo ?b)
;         (unblocked ?b box_0)
;         (unblocked ?b box_1)
;         (unblocked ?b box_2)
;     )
;     :effect (and
;         (not (ready-to-move ?r))
;         (moved-to-object-top ?r ?b)
;     )
; )

(:action move-to-object-front
    :parameters (?r - robot ?b - box)
    :precondition (and
        (free ?r)
        (ready-to-move ?r)
        (todo ?b)
        (unblocked ?b box_0)
        (unblocked ?b box_1)
        (unblocked ?b box_2)
    )
    :effect (and
        (not (ready-to-move ?r))
        (moved-to-object-front ?r ?b)
    )
)

; (:action grasp-top
;     :parameters (?r - robot ?b - box)
;     :precondition (and
;         (free ?r)
;         (moved-to-object-top ?r ?b)
;     )
;     :effect (and
;         (not (free ?r))
;         (not (moved-to-object-top ?r ?b))
;         (ready-to-move ?r)

;         (holding ?r ?b)
;         (unblocked box_0 ?b)
;         (unblocked box_1 ?b)
;         (unblocked box_2 ?b)
;     )
; )

(:action grasp-front
    :parameters (?r - robot ?b - box)
    :precondition (and
        (free ?r)
        (moved-to-object-front ?r ?b)
    )
    :effect (and
        (not (free ?r))
        (not (moved-to-object-front ?r ?b))
        (ready-to-move ?r)

        (holding ?r ?b)
        (unblocked box_0 ?b)
        (unblocked box_1 ?b)
        (unblocked box_2 ?b)
    )
)

(:action move-to-bin
    :parameters (?r - robot ?b - box ?to - bin)
    :precondition (and
        (holding ?r ?b)
        (ready-to-move ?r)
    )
    :effect (and
        (moved-to-bin ?r ?b ?to)
        (not (ready-to-move ?r))
    )
)



(:action release
    :parameters (?r - robot ?b - box ?to - bin)
    :precondition (and
        (moved-to-bin ?r ?b ?to)
    )
    :effect (and
        (free ?r)
        (ready-to-move ?r)
        (not (holding ?r ?b))
        (in ?b ?to)
        (not (moved-to-bin ?r ?b ?to))
        (not (todo ?b))
    )
)


)
