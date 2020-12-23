;Header and description

(define (domain multi_grasp)

;remove requirements that are not needed
(:requirements :strips :typing)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot - object
    stackable_object - object
    box - stackable_object
    red_box - box
    black_box - box
    table - stackable_object
    goal_table - table
)

; un-comment following line if constants are needed
(:constants 
    box_0 - red_box
    box_1 - red_box
    box_2 - black_box
    box_3 - black_box
)

(:predicates ;todo: define predicates here
    (free ?r - robot)
    (holding ?r - robot ?b - box)
    (ready-to-move ?r - robot)

    (moved-to-object ?r - robot ?b - box)
    (moved-to-dest ?r - robot ?b - box ?dest - stackable_object)

    (on ?b - box ?t - stackable_object)

    (unobstructed ?b1 - box ?b2 - box)
    (clear ?des - stackable_object)
)


; (:functions ;todo: define numeric functions here
; )

;define actions here
(:action move-to-object-top
    :parameters (?r - robot ?b - box ?t - table)
    :precondition (and 
        (free ?r)
        (ready-to-move ?r)
        (on ?b ?t)
        (unobstructed ?b box_0)
        (unobstructed ?b box_1)
        (unobstructed ?b box_2)
        (unobstructed ?b box_3)
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
    )
)

(:action move-to-dest
    :parameters (?r - robot ?b - box ?des - stackable_object)
    :precondition (and 
        (holding ?r ?b)
        (ready-to-move ?r)
    )
    :effect (and 
    
    )
)


)