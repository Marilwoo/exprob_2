(define (domain turtlebot)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl)

(:types
	waypoint 
	home
)

(:predicates
	(robot_at ?wp - waypoint)
	(robot_at_home ?h - home)
;	(visited ?wp - waypoint)
;	(visited_home ?h - home)
	(hint_taken ?wp - waypoint)
	(hypo_to_check)
	(hypo_complete)
)


;; Move to any waypoint
(:durative-action goto_waypoint
	:parameters ( ?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?from))
	:effect (and
;		(at end (visited ?to))
		(at end (robot_at ?to))
		(at start (not (robot_at ?from)))
	)
)

;; Move to home
(:durative-action goto_home
	:parameters (?from - waypoint ?to - home)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?from))
	:effect (and
;		(at end (visited_home ?to))
		(at end (robot_at_home ?to))
		(at start (not (robot_at ?from)))
	)
)

;; Move from home
(:durative-action movefrom_home
	:parameters (?h - home ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (at start (robot_at_home ?h))
	:effect (and
;		(at end (visited ?to))
		(at end (robot_at ?to))
		(at start (not (robot_at_home ?h)))
	)
)

;; Taking the hint
(:durative-action take_hint
	:parameters (?wp - waypoint)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?wp))
	:effect (at end (and(hint_taken ?wp)(hypo_to_check)))
)

;; Checking if hypothesis is complete and consistent
(:durative-action check_hint
	:parameters (?h - home)
	:duration ( = ?duration 60)
	:condition (at start (and(hypo_to_check)(robot_at_home ?h)))
	:effect (at end (and(hypo_complete)(not(hypo_to_check))))
)

)
