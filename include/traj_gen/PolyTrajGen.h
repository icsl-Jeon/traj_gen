//
// Created by jbs on 18. 7. 23.
//

#ifndef TRAJ_GEN_POLYTRAJGEN_H
#define TRAJ_GEN_POLYTRAJGEN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <vector>
#include <iostream>
#include <chrono>
#include <memory>

#include "traj_gen/PolySplineXYZ.h" // msg tyype 

#include "qpOASES.hpp"


using namespace Eigen;
using namespace std;
using namespace geometry_msgs;
using namespace traj_gen;
typedef VectorXd TimeSeries;

/**
 *  Lower level routine 
 **/

VectorXd t_vec(int poly_order ,double time ,int n_diff);

    /*
    * scailing matrix D : p(t)=p'D t_vec(tau) where t1<=t<=t2 , 0<=tau<=1
    */
MatrixXd time_scailing_mat(double dt,int poly_order); // consider using sparse matrix

    /*
    *  integral t_vec(n,tau,3)*t_vec(n,tau,3)' from 0 to 1
    */
MatrixXd integral_jerk_squared(int poly_order);

MatrixXd integral_snap_squared(int poly_order);

// find belonging time interval in time series
int find_spline_interval(const vector<double>& knots,double eval_t);

// append mat_sub to mat in the row 
void row_append(MatrixXd & mat,MatrixXd mat_sub);


/**
 *  High level path planner 
 * 
 **/

// trajectory solving mode (referred to README.md)
struct TrajGenOpts{
    
    int objective_derivative; // jerk minimization = 3 , snap minimization = 4
    
    bool is_waypoint_soft;
    double w_d; // weight for deviation

    bool is_single_corridor;  
    bool is_multi_corridor;  

    int poly_order;
    // In case of parallel corridor = true the followings are required
    double safe_r;
    int N_safe_pnts;   

    bool verbose = true; // print the optimization matrices 

};

struct Constraint{

    MatrixXd A;
    MatrixXd b;
};


struct QP_form{
    MatrixXd Q;
    MatrixXd H;
    MatrixXd A;
    MatrixXd b;
    MatrixXd Aeq;
    MatrixXd beq;    
};

struct QP_form_xyz{

    QP_form x;
    QP_form y;
    QP_form z;
};


class PathPlanner{
    private:
        bool is_path_computed;
        bool is_this_verbose = true; 
        nav_msgs::Path current_path; // latest path from optimization for entire horizon 
        traj_gen::PolySplineXYZ spline_xyz; // the coefficient of this polynomials 
        visualization_msgs::Marker safe_corridor_marker; 
        visualization_msgs::Marker knots_marker; // knots marker (the actual point at th knot time)

    public: 
        // constructor
        PathPlanner();
        bool is_spline_valid() {return spline_xyz.is_valid;};        
        // update spline and current path 
        void path_gen(const TimeSeries& knots ,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt );
        QP_form_xyz qp_gen(const TimeSeries& knots ,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt );
        nav_msgs::Path get_path() {return current_path;}
        // evaluate at a time horizon 
        void horizon_eval_spline(int N_eval_interval);
        nav_msgs::Path sub_horizon_eval_spline(int N_eval_interval,double t_start,double t_final);        
        visualization_msgs::Marker get_safe_corridor_marker(){return safe_corridor_marker;}
        visualization_msgs::Marker get_knots_marker();

        // evaluate at a time
        Point point_eval_spline(double t_eval);
        Twist vel_eval_spline(double t_eval);
        Twist accel_eval_spline(double t_eval);

        // ros data retrieving from path update 


        // sub routine 
        PolySpline get_solution(VectorXd sol,int poly_order,int n_seg );        
        VectorXd solveqp(QP_form qp_prob,bool& is_ok);
        Constraint get_init_constraint_mat(double x0, double v0, double a0,TrajGenOpts option); // up to 2nd order conditions 
        Constraint get_continuity_constraint_mat(double dt1,double dt2,TrajGenOpts option); // up to 2nd order continuity 
        
};



#endif //TRAJ_GEN_POLYTRAJGEN_H
