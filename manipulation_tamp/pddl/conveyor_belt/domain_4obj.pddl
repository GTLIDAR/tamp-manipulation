;Header and description

(define (domain conveyor_belt_4obj)

;remove requirements that are not needed
(:requirements :strips :typing)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot - object
    box - object
    normal_box - box
    red_box - normal_box
    black_box - normal_box
    large_box - box
    bin - object
    normal_bin - bin
    far_bin - bin
    conveyor_belt - object
)

; un-comment following line if constants are needed
(:constants 
    box_0 - red_box
    box_1 - black_box
    box_2 - black_box
    box_3 - large_box
    
)

(:predicates ;todo: define predicates here
    (free ?r - robot)
    (holding ?r - robot ?b - box)
    (ready-to-move ?r - robot)

    (moved-to-object ?r - robot ?b - box)
    (moved-to-bin ?r - robot ?b - box ?bin - bin)
    (moved-to-push ?r - robot ?b - box ?bin - bin)

    (on ?b - box ?cb - conveyor_belt)
    (in ?b - box ?bin - bin)

    (unobstructed ?b1 - box ?b2 - box)
)


;define actions here
(:action move-to-object-top
    :parameters (?r - robot ?b - normal_box ?cb - conveyor_belt)
    :precondition (and 
        (free ?r)
        (ready-to-move ?r)
        (on ?b ?cb)

        (unobstructed ?b box_0)
        (unobstructed ?b box_1)
        (unobstructed ?b box_2)
        (unobstructed ?b box_3)
    )
    :effect (and 
        (not (ready-to-move ?r))
        (moved-to-object ?r ?b)
        (not (on ?b ?cb))
    )
)

;define actions here
(:action move-to-object-side
    :parameters (?r - robot ?b - normal_box ?cb - conveyor_belt)
    :precondition (and 
        (free ?r)
        (ready-to-move ?r)
        (on ?b ?cb)

        (unobstructed ?b box_0)
        (unobstructed ?b box_1)
        (unobstructed ?b box_2)
        (unobstructed ?b box_3)
    )
    :effect (and 
        (not (ready-to-move ?r))
        (moved-to-object ?r ?b)
        (not (on ?b ?cb))
    )
)


(:action grasp
    :parameters (?r - robot ?b - normal_box)
    :precondition (and 
        (free ?r)
        (moved-to-object ?r ?b)
    )
    :effect (and 
        (not (free ?r))
        (holding ?r ?b)
        (not (moved-to-object ?r ?b))
        (ready-to-move ?r)

        (unobstructed box_0 ?b)
        (unobstructed box_1 ?b)
        (unobstructed box_2 ?b)
        (unobstructed box_3 ?b)
    )
)

(:action move-to-push
    :parameters (?r - robot ?b - large_box ?bin - normal_bin ?cb - conveyor_belt)
    :precondition (and 
        (free ?r)
        (ready-to-move ?r)
        (on ?b ?cb)

        (unobstructed ?b box_0)
        (unobstructed ?b box_1)
        (unobstructed ?b box_2)
        (unobstructed ?b box_3)
    )
    :effect (and 
        (not (free ?r))
        (not (on ?b ?cb))
        (moved-to-push ?r ?b ?bin)
    )
)


(:action push
    :parameters (?r - robot ?b - large_box ?bin - normal_bin)
    :precondition (and 
        (moved-to-push ?r ?b ?bin)
    )
    :effect (and 
        (free ?r)
        (not (moved-to-push ?r ?b ?bin))
        (in ?b ?bin)
    )
)


(:action move-to-bin
    :parameters (?r - robot ?b - normal_box ?bin - normal_bin)
    :precondition (and 
        (holding ?r ?b)
        (ready-to-move ?r)
    )
    :effect (and 
        (moved-to-bin ?r ?b ?bin)
        (not (ready-to-move ?r))
    )
)

(:action release
    :parameters (?r - robot ?b - normal_box ?bin - normal_bin)
    :precondition (and
        (moved-to-bin ?r ?b ?bin)
    )
    :effect (and
        (free ?r)
        (ready-to-move ?r)
        (not (holding ?r ?b))
        (in ?b ?bin)
        (not (moved-to-bin ?r ?b ?bin))
    )
)

(:action throw
    :parameters (?r - robot ?b - normal_box ?bin - far_bin)
    :precondition (and 
        (holding ?r ?b)
        (ready-to-move ?r)
    )
    :effect (and 
        (free ?r)
        (ready-to-move ?r)
        (not (holding ?r ?b))
        (in ?b ?bin)
    )
)



)
