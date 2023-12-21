(define (domain PACMAN)
	(:requirements :strips :typing)
	(:types
		pacman
		ghost
		food
		location)

	(:predicates
		(connected ?l1 ?l2 - location)
		(pacman-at ?l - location)
		(bomb-at ?l - location)
		(weak ?g - ghost)		(food-at ?l - location)
		(ghost-at ?g - ghost ?l - location)
	)

	(:action move
		:parameters (?p - pacman ?from ?to - location)
		:precondition (and (connected ?from ?to) (pacman-at ?from) (forall (?g - ghost) (not (ghost-at ?g ?to))))
		:effect (and (pacman-at ?to) (not (pacman-at ?from)))
	)

	(:action kill
		:parameters (?p - pacman ?pl - location ?g - ghost ?gl - location)
		:precondition (and (pacman-at ?pl) (connected ?pl ?gl) (ghost-at ?g ?gl) (weak ?g))
		:effect (and (not (pacman-at ?pl)) (pacman-at ?gl) (not (ghost-at ?g ?gl)) (not (weak ?g)))
	)

	(:action power
		:parameters (?p - pacman ?l - location)
		:precondition (and (pacman-at ?l) (bomb-at ?l))
		:effect (and (not (bomb-at ?l)) (forall (?g - ghost) (weak ?g)))
	)

	(:action eat
		:parameters (?p - pacman ?f - food ?l - location)
		:precondition (and (pacman-at ?l) (food-at ?l))
		:effect (and (not (food-at ?l)))
	)

)
