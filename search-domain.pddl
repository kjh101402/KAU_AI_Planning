(define (domain PACMAN)
	(:requirements :strips :typing)
	(:types
		pacman
		food
		location)

	(:predicates
		(connected ?l1 ?l2 - location)
		(pacman-at ?l - location)
		(food-at ?l - location)
	)

	(:action move
		:parameters (?p - pacman ?from ?to - location)
		:precondition (and (connected ?from ?to) (pacman-at ?from))
		:effect (and (pacman-at ?to) (not (pacman-at ?from)))
	)

	(:action eat
		:parameters (?p - pacman ?f - food ?l - location)
		:precondition (and (pacman-at ?l) (food-at ?l))
		:effect (and (not (food-at ?l)))
	)

)
