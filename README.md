# traj_gen 

<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/intro.png">

## USAGE 

This library provides interface where you can specifiy a sequence of waypoints from Rviz 

Also, it can be used as components by other packages 

## Alogrithm 

Minimum jerk or snap with motion primitives of polynomials 


1. Waypoints 

(1) Soft waypoints

	not necessarily pass through the specified waypoints 

(2) Hard waypoints
	
	the waypoints will be passed exactly as hard constraints 

2. Corridor
 
(1) multiple sub boxes between waypoints which is axis-parallel 
	
	Number of constraints will be increased but x,y,z can be solved independently.
	
	In general, imposing too many sub constraints will be infeasible for polynomial curves 

(2) single box between waypoints 

	Number of constraints will be decreased but x,y,z can be solved independently
	
	
 	
