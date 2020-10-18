;Header and description

(define (domain stationary_object_sorting)

;remove requirements that are not needed
(:requirements :strips :typing)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot - object
    box - object
    red_box - box
    black_box - box
    table - object
    goal_table - table
    free_table - table
    init_table - table
)

; un-comment following line if constants are needed
(:constants 
    box_0 - red_box
    box_1 - black_box
    box_2 - black_box
    box_3 - black_box
    box_4 - black_box
)

(:predicates ;todo: define predicates here
    (free ?r - robot)
    (holding ?r - robot ?b - box)
    (ready-to-move ?r - robot)

    (moved-to-object ?r - robot ?b - box)
    (moved-to-table ?r - robot ?b - box ?t - table)

    (on ?b - box ?t - table)

    (unobstructed ?b1 - box ?b2 - box)

)


; (:functions ;todo: define numeric functions here

; )

;define actions here
(:action move-to-object-top
    :parameters (?r - robot ?b - box ?t - init_table)
    :precondition (and 
        (free ?r)
        (ready-to-move ?r)
        (on ?b ?t)

        (unobstructed ?b box_0)
        (unobstructed ?b box_1)
        (unobstructed ?b box_2)
        (unobstructed ?b box_3)
        (unobstructed ?b box_4)
    )
    :effect (and 
        (not (ready-to-move ?r))
        (moved-to-object ?r ?b)
        (not (on ?b ?t))
    )
)

(:action move-to-object-side
    :parameters (?r - robot ?b - box ?t - init_table)
    :precondition (and 
        (free ?r)
        (ready-to-move ?r)
        (on ?b ?t)

        (unobstructed ?b box_0)
        (unobstructed ?b box_1)
        (unobstructed ?b box_2)
        (unobstructed ?b box_3)
        (unobstructed ?b box_4)
    )
    :effect (and 
        (not (ready-to-move ?r))
        (moved-to-object ?r ?b)
        (not (on ?b ?t))
    )
)

(:action grasp
    :parameters (?r - robot ?b - box)
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
        (unobstructed box_4 ?b)
    )
)

(:action move-to-goal-table
    :parameters (?r - robot ?b - red_box ?t - goal_table)
    :precondition (and 
        (holding ?r ?b)
        (ready-to-move ?r)
    )
    :effect (and 
        (moved-to-table ?r ?b ?t)
        (not (ready-to-move ?r))
    )
)

(:action move-to-free-table
    :parameters (?r - robot ?b - black_box ?t - free_table)
    :precondition (and 
        (holding ?r ?b)
        (ready-to-move ?r)
    )
    :effect (and 
        (moved-to-table ?r ?b ?t)
        (not (ready-to-move ?r))
    )
)

(:action release
    :parameters (?r - robot ?b - box ?t - table)
    :precondition (and
        (moved-to-table ?r ?b ?t)
    )
    :effect (and
        (free ?r)
        (ready-to-move ?r)
        (not (holding ?r ?b))
        (on ?b ?t)
        (not (moved-to-table ?r ?b ?t))
    )
)


)
