## traj_gen :  a continuous trajectory generation with simple API

<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen2/blob/master/img/tutorial.gif">
</p>

*traj_gen* is a continuous trajectory generation package where <u>high order derivatives</u> 
along the trajectory are minimized while satisfying waypoints (equality) and axis-parallel box constraint (inequality). The objective and constraints are formulated in *quadratic programming* (QP) to cater the real-time performance. 

- To parameterize a trajectory, we use two types of curve: 1) **piecewise-polynomials** [1,2] and 2) **a sequence of points** [3]. 
The difference is optimization variables.   

  a. **piecewise-polynomials (polyTrajGen class)** : It defines the primitive of the curve as polynomical spline. The optimization target is either *polynomial coefficients* [1] or *free end-derivatives* of spline segments [2] (can be set in constructor). In general, the latter has fewer optimization variables as it reduces the number of variable as much as the number of equality constraints.    
  b. **a sequence of points (optimTrajGen class)** : It does not limit the primitive of the curve. The optimization target is a finite set of points. The final curve is defined as a linear interpolant of the set of points. The point density (# of points per time) should be set in the constructor. Instead of unlimited representation capability of a curve, the size of optimization is driectly affected by the point density.          
  
- In this package, we use **pin** to accommodate the two constraints: equality (*fix pin*) and inequality (*loose pin*). Pin can be imposed regardless of the order of derivatives. *Fix-pin* refers a waypoint constraint, 
and *loose-pin* denotes a axis-parallel box constraint. The pin is a triplets (time (t), order of derivatives (d), value (x)) where x is 
a vector in case of fix-pin while two vectors [xl xu] for the loose-pin.  

 - We implemented traj_gen in Matlab and C++(upcoming ~ the end of Mar). In case of 2D trajectory generation in Matlab, we provide interactive pin selection (see poly_example/main2D).
Also, we plan to provide ROS support such as the [previous version](https://github.com/icsl-Jeon/traj_gen)


### Matlab API quick start (check multiple examples in poly_example and optimal_example)
<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen2/blob/master/img/quick_start.png">
</p>


  ### Reference 

[1] Mellinger, Daniel, and Vijay Kumar. "Minimum snap trajectory generation and control for quadrotors." *2011 IEEE International Conference on Robotics and Automation*. IEEE, 2011.

[2] Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." *Robotics Research*. Springer, Cham, 2016. 649-666.

[3] Ratliff, Nathan, et al. "CHOMP: Gradient optimization techniques for efficient motion planning." *2009 IEEE International Conference on Robotics and Automation*. IEEE, 2009.
