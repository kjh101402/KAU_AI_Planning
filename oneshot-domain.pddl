(define (domain PACMAN)
	(:requirements :strips :typing)
	(:types
		pacman
		location)

	(:predicates
		(connected ?l1 ?l2 - location)
		(pacman-at ?l - location)
		(visited ?p - pacman ?l - location)
	)

	(:action move
		:parameters (?p - pacman ?from ?to - location)
		:precondition (and  (connected ?from ?to) (pacman-at ?from) (not (visited ?p ?to)))
		:effect (and (pacman-at ?to) (not (pacman-at ?from)) (visited ?p ?to))
	)
 
)
