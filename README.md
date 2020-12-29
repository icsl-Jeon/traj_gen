<img src= "https://github.com/icsl-Jeon/traj_gen/blob/master/img/logo2.png" width="300" >

## traj_gen :  a continuous trajectory generation with simple API (C++/Matlab/Python)
###   Version 2.1.0  (Mar 22, 2020) 
<img src = "https://travis-ci.com/icsl-Jeon/traj_gen.svg?branch=master"> <img src = "https://img.shields.io/github/license/Naereen/StrapDown.js.svg">
<img src = "https://img.shields.io/github/v/release/icsl-Jeon/traj_gen?style=plastic">

[![View traj_gen-matlab on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://kr.mathworks.com/matlabcentral/fileexchange/74573-traj_gen-matlab)



<p align = "center">
<img src= "https://github.com/icsl-Jeon/traj_gen/blob/master/img/tutorial.gif">
</p> 
*traj_gen* is a continuous trajectory generation package where <u>high order derivatives</u> 
along the trajectory are minimized while satisfying waypoints (equality) and axis-parallel box constraint (inequality). The objective and constraints are formulated in *quadratic programming* (QP) to cater the real-time performance. The whole code is written in C++ and Matlab (go to [submodule](https://github.com/icsl-Jeon/traj_gen-matlab) for its API). The C++ API was tested in Ubuntu 14.04/16.04/18.04. Especially, if you are looking for a off-the-shelf ROS implementation, checkout ros branch

- To parameterize a trajectory, we use two types of curve: 1) **piecewise-polynomials** [1,2] and 2) **a sequence of points** [3]. 
The difference is optimization variables.   

  a. **piecewise-polynomials (polyTrajGen class)** : It defines the primitive of the curve as polynomical spline. The optimization target is either *polynomial coefficients* [1] or *free end-derivatives* of spline segments [2] (can be set in constructor). In general, the latter has fewer optimization variables as it reduces the number of variable as much as the number of equality constraints.    
  b. **a sequence of points (optimTrajGen class)** : It does not limit the primitive of the curve. The optimization target is a finite set of points. The final curve is defined as a linear interpolant of the set of points. The point density (# of points per time) should be set in the constructor. Instead of unlimited representation capability of a curve, the size of optimization is driectly affected by the point density.          
  
- In this package, we use **pin** to accommodate the two constraints: equality (*fix pin*) and inequality (*loose pin*). Pin can be imposed regardless of the order of derivatives. *Fix-pin* refers a waypoint constraint, 
and *loose-pin* denotes a axis-parallel box constraint. The pin is a triplets (time (t), order of derivatives (d), value (x)) where x is 
a vector in case of fix-pin while two vectors [xl xu] for the loose-pin.  

 - We implemented traj_gen in Matlab and C++. In case of 2D trajectory generation in Matlab, we provide interactive pin selection (see poly_example/main2D).
Also, we plan to provide ROS support (~end of Mar) such as the [previous version](https://github.com/icsl-Jeon/traj_gen)


## Getting started 

### 1. Installation 
- **Setting dependencies** 
  #### (1) Eigen 3  
  ```
  $ sudo apt-get update
  $ sudo apt-get install libeigen-dev
  ```
  #### (2) qpOASES (note -fPIC flag when cmake)
  ```
  $ git clone https://github.com/coin-or/qpOASES.git
  $ cd path/to/qpOASES
  $ mkdir build && cd build
  $ cmake .. -DCMAKE_CXX_FLAGS=-fPIC
  $ sudo make install
  ```
- **Build traj_gen (C++)**
  ```
  $ git clone https://github.com/icsl-Jeon/traj_gen.git
  $ cd ./traj_gen/cpp
  $ mkdir build && cd build
  $ cmake ..
  & make && sudo make install
  ```
- **Testing the package**
  ```
  $ cd ./traj_gen/cpp/test
  $ mkdir build && cd build
  $ cmake ..
  $ make 
  $ ./test_pin 
  ```
### 2. C++ API quick start 

### (1) Basic example (PolyTrajGen class) 

```cpp
#include <traj_gen2/TrajGen.hpp>
#include <iostream>
using namespace trajgen;
using namespace Eigen;

int main(){
    // 1. Prameter setting
    const int dim = 3; uint poly_order = 8, max_conti = 4;
    time_knots ts{0,2,4,7};
    PolyParam pp(poly_order,max_conti,ALGORITHM::POLY_COEFF);
    Vector3f objWeights(0,1,1);
    PolyTrajGen<dim> pTraj(ts,pp);

    // 2. Pin
    // 2.1 FixPin
    FixPin<dim> x1(0.0f,0,Vector3f(0,0,0));
    FixPin<dim> x2(2.0f,0,Vector3f(2,-1,2));
    FixPin<dim> x3(4.0f,0,Vector3f(5,3,4));
    FixPin<dim> x4(7.0f,0,Vector3f(7,-5,5));
    std::vector<Pin<dim>*> pinSet{&x1,&x2,&x3,&x4}; // to prevent downcasting slicing, vector of pointers
    pTraj.addPinSet(pinSet);

    FixPin<dim> xdot0(0.0f,1,Vector3f(0,0,0));
    FixPin<dim> xddot0(0.0f,2,Vector3f(0,0,0));
    pTraj.addPin(&xdot0); pTraj.addPin(&xddot0);

    // 2.2 LoosePin
    LoosePin<dim> passCube(3.0f,0,Vector3f(3,-3,1),Vector3f(4.2,-2,2));
    pTraj.addPin(&passCube);

    // 3. Solve
    pTraj.setDerivativeObj(objWeights); bool verbose = false;
    bool isSolved = pTraj.solve(verbose);

    // 4. Evaulate the curve
    float t_eval = 3,d_eval = 1;
    Vector3f xdot_eval = pTraj.eval(t_eval,d_eval);
    if (isSolved)
        cout << xdot_eval << endl;
    return 0; 

}

```



### (2) Advanced example (PolyTrajGen + OptimTrajGen) 

```cpp
#include <traj_gen2/TrajGen.hpp>
#include <iostream>
#include <chrono>

using namespace trajgen;
using namespace Eigen;
using Type = double ;

int main(){
    // 1. Prameter setting
    // common
    const int dim = 3;
    time_knots<Type> ts{0,2,4,7};
    Vector<Type,3> objWeights(0,1,1);

    // polyTrajGen
    uint poly_order = 8, max_conti = 4;
    PolyParam pp(poly_order,max_conti,ALGORITHM::END_DERIVATIVE); // or ALGORITHM::POLY_COEFF
    // optimTrajGen
    Type pntDensity = 5;

    // 2. Pin
    // 2.1 FixPin
    FixPin<Type,dim> x1(0.0f,0,Vector<Type,dim>(0,0,0));
    FixPin<Type,dim> x2(2.0f,0,Vector<Type,dim>(2,-1,2));
    FixPin<Type,dim> x3(4.0f,0,Vector<Type,dim>(5,3,4));
    FixPin<Type,dim> x4(7.0f,0,Vector<Type,dim>(7,-5,5));

    FixPin<Type,dim> xdot0(0.0f,1,Vector<Type,dim>(0,0,0));
    FixPin<Type,dim> xddot0(0.0f,2,Vector<Type,dim>(0,0,0));

    // 2.2 LoosePin
    LoosePin<Type,dim> passCube(3.0f,0,Vector<Type,dim>(3,-3,1),Vector<Type,dim>(4.2,-2,2));

    std::vector<Pin<Type,dim>*> pinSet{&x1,&x2,&x3,&x4,&xdot0,&xddot0,&passCube}; // to prevent downcasting slicing, vector of pointers

    // Let's test the two trajGen class
    TrajGen<Type,dim>** pTrajGenSet = new TrajGen<Type,dim>*[2];
    pTrajGenSet[0] = new PolyTrajGen<Type,dim>(ts,pp);
    pTrajGenSet[1] = new OptimTrajGen<Type,dim>(ts,pntDensity);
    bool verbose = false;
    bool isSolved = false;
    string TrajGenClass[2] = {"PolyTraj","OptimTraj"};

    Type t_eval = 3; d_order d_eval = 1;

    for (uint i = 0 ; i <2 ; i++) {
        auto begin = std::chrono::steady_clock::now();
        pTrajGenSet[i]->setDerivativeObj(objWeights);
        pTrajGenSet[i]->addPinSet(pinSet);
        isSolved = pTrajGenSet[i]->solve(verbose);
        printf("For %s, The evaluated value for (t=%.2f, d=%d) : ",TrajGenClass[i].c_str(),t_eval,d_eval);
        if (isSolved)
            cout << pTrajGenSet[i]->eval(t_eval,d_eval).transpose() << endl;
        else
            cout << "Failed. " << endl;

        auto end= std::chrono::steady_clock::now();

        std::cout << "Total time :  " <<
        std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()*1e-3 << "[ms]" << std::endl;
    }
    return 0;

}

```



### 3. Using traj_gen in your cmake project (CMakeLists.txt)

```
find_package(TrajGen REQUIRED)
...
add_executable(your_exe ${YOUR_SOURCE})
target_link_libraries( your_exe .... traj_gen )

```




## Detailed description on API

### Parameters (arguments in constructor and setDerivativeObj method)
- **Common** 
  - *Knots (t1,...,tM)* : time knots. In case of polyTrajGen, it defines the segment intervals in time domain. *The fix-pin can be imposed on these time knots* (no limitation for loose pin). In case of optimTrajGen, the time knot is just the start time and end time as it is not defined on a set of time sgements. 
  - *Penality weights for the integral of derivative squared* : as the objective of our optimization is the weighted sum of the integral of derivative squared, we have to define the importance weight for each component. This value ws = [w1 w2 ... wd] can be set as the argument of setDerivativeObj(ws). For example, if you want to implement a minimum snap trajectory generation, then set ws = [0 0 0 1] while ws = [0 0 1] for minimum jerk trajectory generation.
  
- **polyTrajGen**
  - *Polynomial order (N)* : the order of all the polynomial segments. Although it can increase the power of representation of a curve, the size of optimization variables increases in proportion to (N x M).   
  
  - *Optimization target* (*'end-derivative'* or *'poly-coeff'*) : the target of optimization. 'poly-coeff' method sets the coefficients of polynomials as optimization variables in a similar way with [1]. The 'end-derivative' sets the optimization variables as the free derivative values of each end point on a polynomial segment. As this method is equivalent to plugging the equality constraints (fix pin and continuity) to optimization problem, it reduces the optimization dimension at the cost of inversion of a mapping matrix. The dof of a segment is thus (poly_order - # of fix pins on the segment - maximal continuity). For the details, please refer [2].   
    
  - *Maximum continuity* : the maximally imposed continuity order between neighboring segements. Higher value of this parameter enhances the quality of smoothness. However, too high value of this value restricts the dof for optimization, downgrading the opitimization result.     
  
- **optimTrajGen**
   - *Point density* : the number of posed points per time [s]. For long-ranged trajectory, thus, the total number of variables will increase leading to the burden of the optimization. 

### Public methods

- Please run the following command to open the dedicated doxygen: 

  ```
   firefox cpp/docs/html/index.html
  ```

  

  ### Reference 

[1] Mellinger, Daniel, and Vijay Kumar. "Minimum snap trajectory generation and control for quadrotors." *2011 IEEE International Conference on Robotics and Automation*. IEEE, 2011.

[2] Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." *Robotics Research*. Springer, Cham, 2016. 649-666.

[3] Ratliff, Nathan, et al. "CHOMP: Gradient optimization techniques for efficient motion planning." *2009 IEEE International Conference on Robotics and Automation*. IEEE, 2009.
