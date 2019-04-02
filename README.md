# traj_gen 

<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/intro.png"> 

**(left)** piecewise polynomial path obtained **(right)** multiple safe corridors in subinterval 

## USAGE 

### Qt gui
<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/traj_gen.png"> 

This library provides interface where you can specifiy a sequence of waypoints from Rviz 

(1) ROS connect : please push the button at the beginning while roscore is running 

(2) select waypoints : waypoints insertion from rviz is allowed while this button is clicked 

(3) trajectory generation : quadratic programming with assigned parameters

(4) publish : the time allocation of the trajectory is equal division from 0 to "simulation tf" of gui. A desired control point will be published in *geometry_msg/PoseStamped* message type.  



### waypoints selection from user
<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/traj_gen-2.png"> 

*You can also save and load the waypoints in txt file format. In that way, you may assign the heights for each waypoint*

## Alogrithm 

This package is based on minimum jerk or snap with motion primitives of polynomials 

**refer**
Mellinger, Daniel, and Vijay Kumar. "Minimum snap trajectory generation and control for quadrotors." 2011 IEEE International Conference on Robotics and Automation. IEEE, 2011.
* * * 
1. Waypoints 


<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/hard_vs_soft.png"> 

(1) Soft waypoints

not necessarily pass through the specified waypoints. But it can minimize jerk more.

(2) Hard waypoints
	
the waypoints will be passed exactly as hard constraints 

* * * 

2. Corridor
 
<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/explain_corridor.jpg"> 

(1) multiple sub boxes between waypoints which is axis-parallel 
	
Number of constraints will be increased but x,y,z can be solved independently.
	
In general, imposing too many sub constraints will be infeasible for polynomial curves 

(2) single box between waypoints 

Number of constraints will be decreased but x,y,z can be solved independently
	
	
 	

