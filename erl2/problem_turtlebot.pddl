(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 - waypoint
    wphome - home
)
(:init
    (robot_at_home wphome)
;    (visited_home wphome)
	(not(hint_taken wp0))
	(not(hint_taken wp1))
	(not(hint_taken wp2))
	(not(hint_taken wp3))
	(not(hypo_complete))
	(not(hypo_to_check))
;	(not(visited wp0))
;	(not(visited wp1))
;	(not(visited wp2))
;	(not(visited wp3))
	
	
)
(:goal (and
;   (visited_home wphome)
;    (visited wp0)
;    (visited wp1)
;    (visited wp2)
;    (visited wp3)
    (hint_taken wp0)
    (hint_taken wp1)
    (hint_taken wp2)
    (hint_taken wp3)
;    (hypo_to_check)
    (hypo_complete)
)))
