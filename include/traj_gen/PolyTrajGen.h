//
// Created by jbs on 18. 7. 23.
//

#ifndef TRAJ_GEN_POLYTRAJGEN_H
#define TRAJ_GEN_POLYTRAJGEN_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <vector>
#include <iostream>
#include <chrono>
#include <memory>

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>

#include "traj_gen/PolySplineXYZ.h" // msg tyype 
#include "traj_gen/SplineGen.h"


using namespace Eigen;
using namespace std;
#define CGAL_OPTI_MODE 0
#define CVXGEN_OPTI_MODE 1
typedef int MODE;
typedef double Time;
typedef int PolyOrder;
typedef int N_diff;

// program and solution types
typedef CGAL::Quadratic_program_from_iterators
<float**,                                                // for A
 float*,                                                 // for b
 CGAL::Const_oneset_iterator<CGAL::Comparison_result>, // for r
 bool*,                                                // for fl
 float*,                                                 // for l
 bool*,                                                // for fu
 float*,                                                 // for u
 float**,                                                // for D
 float*>                                                 // for c 
CGAL_Program;
typedef CGAL::MP_Float ET;
typedef CGAL::Quadratic_program_solution<ET> CGAL_Solution;
typedef float Weight; // weight

using namespace traj_gen;
namespace TrajGen {


    class traj_gen_server{

        public:
            ros::ServiceServer server; 

            traj_gen_server();
            bool traj_gen_callback( SplineGenRequest& , SplineGenResponse& );

        
    };

    /*
     * TYPEDEF
     */
    typedef VectorXd TimeSeries;
    
    // jerk minimization with waypoint cost ( not hard constraint )
    // remember : if this is solved with cvxgen, the number of var is fixed     
    PolySplineXYZ min_jerk_soft(const TimeSeries&,const nav_msgs::Path&,const geometry_msgs::Twist&,Weight,MODE);
    // evaluation of spline_path with given number of interval points : for the purpose of vizualization
    nav_msgs::Path horizon_eval_spline(PolySplineXYZ &,int);
    // for actual travel, this function will be used
    geometry_msgs::Point point_eval_spline(PolySplineXYZ &, ros::Time );



    /*TrajGen::

     * p(t)=t_vec(n_order,t,0)'p=p't_vec(n_order,t,0)
     * dot{p(t)}=t_vec(n_order,t,1)'p=p't_vec(n_order,t,1)
     * ...
     */
    VectorXd t_vec(PolyOrder ,Time,N_diff);

    /*
     * scailing matrix D : p(t)=p'D t_vec(tau) where t1<=t<=t2 , 0<=tau<=1
     */
    MatrixXd time_scailing_mat(Time,Time,PolyOrder); // consider using sparse matrix

    /*
     *  integral t_vec(n,tau,3)*t_vec(n,tau,3)' from 0 to 1
     */
    MatrixXd integral_jerk_squared(PolyOrder);


    // find belonging time interval in time series
    Index find_spline_interval(const vector<double>&,ros::Time);
}


// CVXGEN : deprecated.. 

// namespace CVXGEN_OPTI{
//     void construct_qp(Params& ,MatrixXd&,MatrixXd&,MatrixXd&,MatrixXd&);
//     TrajGen::PolySpline get_solution(Vars&,int ,int );
// }

// CGAL
namespace CGAL_OPTI{
    CGAL_Solution solveqp(MatrixXd&,MatrixXd&,MatrixXd&,MatrixXd&);
    PolySpline get_solution(CGAL_Solution,int,int);
}



#endif //TRAJ_GEN_POLYTRAJGEN_H
