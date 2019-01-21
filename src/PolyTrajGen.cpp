//
// Created by jbs on 18. 7. 23.
//

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SparseCore>
#include "traj_gen/PolySpline.h"



// extern Params params;

using namespace Eigen;
using namespace std;

namespace CVXGEN_OPTI{
    void construct_qp(Params &params, MatrixXd & Q, MatrixXd & H, MatrixXd & Aeq, MatrixXd & beq) {

        // objective function

        int n_var=Q.rows();
        int n_constraint=Aeq.rows();

        if (n_var*n_var==324)
            for(int r=0;r<n_var;r++) {
                params.H[r]=H.coeff(0,r);
                for (int c = 0; c < n_var; c++)
                    params.Q[r + n_var * c] = Q.coeff(r, c);
            }
        else
            printf("ERROR: size mismatch with CVXGEN\n");

        for (int r=0;r<n_constraint;r++) {
            for (int c = 0; c < n_var; c++)
                params.Aeq[r + n_constraint * c] = Aeq.coeff(r, c);
            params.beq[r]=beq.coeff(r,0);
        }

    }

    TrajGen::PolySpline get_solution(Vars & var,  int poly_order, int n_seg ) {

        TrajGen::PolySpline polySpline;
        polySpline.n_seg=n_seg;
        polySpline.poly_order=poly_order;
        int n_var=n_seg*(poly_order+1);

        // from lowest order 0
        for(int n=0;n<n_seg;n++){
            TrajGen::PolyCoeff coeff(poly_order+1);
            for(int i=0;i<poly_order+1;i++)
                coeff.coeffRef(i)=var.x[n*(poly_order+1)+i];
            polySpline.spline.push_back(coeff);
        }

        return polySpline;
    }

}
// memory leak may occur. should be revised 
namespace CGAL_OPTI{
    CGAL_Solution solveqp(MatrixXd& Q,MatrixXd& H,MatrixXd& Aeq,MatrixXd& beq){
        // because equlity 
        CGAL::Const_oneset_iterator<CGAL::Comparison_result>  r(CGAL::EQUAL);

        // phase 1: construct 
        int N_var = Q.cols();
        int N_const = Aeq.rows();
        MatrixXf Qsym = ((Q + Q.transpose())/2).array().cast<float>();
        // make pointer for it and assign memory  
        shared_ptr<float*> Aarr(new float*[N_var]);
        shared_ptr<float> barr(new float[N_const]);        
        shared_ptr<float*> Qarr(new float*[N_var]);
        shared_ptr<float> Harr(new float[N_var]);
        shared_ptr<float> lbarr(new float[N_var]);
        shared_ptr<float> ubarr(new float[N_var]);
        shared_ptr<bool> fuarr(new bool[N_var]);
        shared_ptr<bool> flarr(new bool[N_var]);

        for(int r = 0; r<N_const ; r++)
            Aarr.get()[r] = new float[N_const];
        for(int r = 0; r<N_var;r++)
            Qarr.get()[r] = new float[N_var];

        // reset the element of pointer 
        
        // for constraints 
        for (int r = 0;r<N_const;r++){
            barr.get()[r] = beq(r);            
            for (int c = 0;c<N_var;c++)
                Aarr.get()[c][r] = Aeq(r,c);
            
        }

        // for objective and flags
        for (int r = 0;r<N_var;r++){
            Harr.get()[r] = H(r,0);    
            lbarr.get()[r] = 0; ubarr.get()[r] = 0; // dummy      
            flarr.get()[r] = false; fuarr.get()[r] = false; // dummy       
 
            for (int c = 0;c<N_var;r++)
                Qarr.get()[c][r] = Q(r,c);            
        }

        CGAL_Program qp(N_var,N_const,Aarr.get(),barr.get(),r,
                flarr.get(),lbarr.get(),fuarr.get(),ubarr.get(),
                Qarr.get(),Harr.get(),0);
        
        CGAL_Solution s = CGAL::solve_quadratic_program(qp,ET());
        
    }

    TrajGen::PolySpline get_solution(CGAL_Solution s,int poly_order,int n_seg){

        TrajGen::PolySpline polySpline;
        // polySpline.n_seg=n_seg;
        // polySpline.poly_order=poly_order;
        // int n_var=n_seg*(poly_order+1);

        // VectorXf var(n_var);
        // int idx = 0;
        // for(auto it = s.variable_values_begin();it!=s.variable_values_end();it++,idx++){
        //     var(idx) = it->num.to_double()/it->den.to_double();
        // }

        // // from lowest order 0
        // for(int n=0;n<n_seg;n++){
        //     TrajGen::PolyCoeff coeff(poly_order+1);
        //     for(int i=0;i<poly_order+1;i++)
        //         coeff.coeffRef(i)=var(n*(poly_order+1)+i);
        //     polySpline.spline.push_back(coeff);
        // }

        return polySpline;

    }
}




TrajGen::PolySplineXYZ TrajGen::min_jerk_soft(const TimeSeries & ts, const nav_msgs::Path & pathPtr, const geometry_msgs::Twist &pointPtr,
                                              Weight w_j,MODE mode) {
    // mode 
    
    
    /*
     * len(ts)=len(path)
     */
    int n_seg = pathPtr.poses.size() - 1;

    PolyOrder poly_order = 5; // 5th order is enough
    const int blck_size=poly_order+1;

    /*
     *  cost : p'Qp+Hp
     */

    int n_var_total = (poly_order + 1) * n_seg;

    // jerk cost matrix

    MatrixXd Q_jerk(n_var_total,n_var_total);
    Q_jerk.setZero();

    for (int n = 0; n < n_seg; n++) {
        MatrixXd Dn = time_scailing_mat(ts[n], ts[n + 1], poly_order);
        Time dn = ts[n + 1] - ts[n];
        Q_jerk.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=Dn*integral_jerk_squared(poly_order)*Dn/pow(dn,5);

    }

    // soft waypoint constraint (objective function)
    MatrixXd Q_wpnt(n_var_total,n_var_total); Q_wpnt.setZero();
    MatrixXd H_wpnt_x(1,n_var_total); H_wpnt_x.setZero();
    MatrixXd H_wpnt_y(1,n_var_total); H_wpnt_y.setZero();
    MatrixXd H_wpnt_z(1,n_var_total); H_wpnt_z.setZero();

    for(int n=0;n<n_seg;n++){
        MatrixXd Dn = time_scailing_mat(ts[n], ts[n + 1], poly_order);
        int insert_start=blck_size*(n);

        Q_wpnt.block(insert_start,insert_start,blck_size,blck_size)=Dn*t_vec(poly_order,1,0)*t_vec(poly_order,1,0).transpose()*Dn;
        H_wpnt_x.block(0,insert_start,1,blck_size)=-2*(pathPtr.poses[n+1].pose.position.x)*t_vec(poly_order,1,0).transpose()*Dn;
        H_wpnt_y.block(0,insert_start,1,blck_size)=-2*(pathPtr.poses[n+1].pose.position.y)*t_vec(poly_order,1,0).transpose()*Dn;
        H_wpnt_z.block(0,insert_start,1,blck_size)=-2*(pathPtr.poses[n+1].pose.position.z)*t_vec(poly_order,1,0).transpose()*Dn;

    }

    /*
     *  Equality Constraints
     */


    // number of constraints
    int n_init_constraint=2;
    int n_0_constraint=n_seg-1;
    int n_1_constraint=n_seg-1;
    int n_2_constraint=n_seg-1;
    int n_constraint=n_init_constraint+n_0_constraint+n_1_constraint+n_2_constraint; // total number of equlity constraint

    MatrixXd Aeq(n_constraint,n_var_total),beqx(n_constraint,1),beqy(n_constraint,1),beqz(n_constraint,1);
    Aeq.setZero(); beqx.setZero(); beqy.setZero(); beqz.setZero();

    // initial constraints  // x0,x0dot

    Aeq.coeffRef(0,0)=1; Aeq.coeffRef(1,1)=1;
    beqx.coeffRef(0,0)=pathPtr.poses[0].pose.position.x;
    beqy.coeffRef(0,0)=pathPtr.poses[0].pose.position.y;
    beqz.coeffRef(0,0)=pathPtr.poses[0].pose.position.z;

    beqx.coeffRef(1,0)=pointPtr.linear.x;
    beqy.coeffRef(1,0)=pointPtr.linear.y;
    beqz.coeffRef(1,0)=pointPtr.linear.z;

    // 0th order continuity
    int row_insert_idx=2; int col_insert_idx1=0; int col_insert_idx2=col_insert_idx1+blck_size;
    for(int n=0;n<n_seg-1;n++,row_insert_idx++){
        MatrixXd Dn=time_scailing_mat(ts[n],ts[n+1],poly_order);
        MatrixXd Dn_1=time_scailing_mat(ts[n+1],ts[n+2],poly_order);

        Aeq.block(row_insert_idx,col_insert_idx1,1,blck_size)=t_vec(poly_order,1,0).transpose()*Dn;
        Aeq.block(row_insert_idx,col_insert_idx2,1,blck_size)=-t_vec(poly_order,0,0).transpose()*Dn_1;
        col_insert_idx1=col_insert_idx2; col_insert_idx2=col_insert_idx1+blck_size;
    }


    // 1st order continuity
    col_insert_idx1=0; col_insert_idx2=col_insert_idx1+blck_size;
    for(int n=0;n<n_seg-1;n++,row_insert_idx++){
        MatrixXd Dn=time_scailing_mat(ts[n],ts[n+1],poly_order);
        MatrixXd Dn_1=time_scailing_mat(ts[n+1],ts[n+2],poly_order);
        Time dn=ts[n+1]-ts[n];
        Time dn_1=ts[n+2]-ts[n+1];

        Aeq.block(row_insert_idx,col_insert_idx1,1,blck_size)=t_vec(poly_order,1,1).transpose()*Dn/dn;
        Aeq.block(row_insert_idx,col_insert_idx2,1,blck_size)=-t_vec(poly_order,0,1).transpose()*Dn_1/dn_1;
        col_insert_idx1=col_insert_idx2; col_insert_idx2=col_insert_idx1+blck_size;
    }

    // 2nd order continuity
    col_insert_idx1=0; col_insert_idx2=col_insert_idx1+blck_size;
    for(int n=0;n<n_seg-1;n++,row_insert_idx++){
        MatrixXd Dn=time_scailing_mat(ts[n],ts[n+1],poly_order);
        MatrixXd Dn_1=time_scailing_mat(ts[n+1],ts[n+2],poly_order);
        Time dn=ts[n+1]-ts[n];
        Time dn_1=ts[n+2]-ts[n+1];

        Aeq.block(row_insert_idx,col_insert_idx1,1,blck_size)=t_vec(poly_order,1,2).transpose()*Dn/pow(dn,2);
        Aeq.block(row_insert_idx,col_insert_idx2,1,blck_size)=-t_vec(poly_order,0,2).transpose()*Dn_1/pow(dn_1,2);
        col_insert_idx1=col_insert_idx2; col_insert_idx2=col_insert_idx1+blck_size;
    }
//
//    /*
//     * QP solve : p'Qp+Hp s.t Aeq=beq
//     */
//
//    std::cout<<"Aeq"<<std::endl;
//    std::cout<<Aeq<<std::endl;
//
//    std::cout<<"beq"<<std::endl;
//    std::cout<<beqx<<std::endl;
//
//
//    std::cout<<"Q_jerk"<<std::endl;
//    std::cout<<Q_jerk<<std::endl;
//
//    std::cout<<"Q_wpnt"<<std::endl;
//    std::cout<<Q_wpnt<<std::endl;
//
//
//    std::cout<<"H_wpnts"<<std::endl;
//    std::cout<<H_wpnt_x<<std::endl;
//    std::cout<<H_wpnt_y<<std::endl;

    MatrixXd Q=w_j*Q_jerk+Q_wpnt;

//    std::cout<<Q<<std::endl;

   // std::cout<<"[CVXGEN] solution: "<<std::endl;


    PolySpline spline_x,spline_y,spline_z;
    // solve for x
    auto t0 = std::chrono::high_resolution_clock::now();
    if(mode == CVXGEN_OPTI_MODE){
        CVXGEN_OPTI::construct_qp(params,Q,H_wpnt_x,Aeq,beqx);
        solve();
        spline_x=CVXGEN_OPTI::get_solution(vars,poly_order,n_seg);
    }else if (mode == CGAL_OPTI_MODE){

        spline_x = CGAL_OPTI::get_solution (CGAL_OPTI::solveqp(Q,H_wpnt_x,Aeq,beqx),poly_order,n_seg);

    }else{
        cerr<<"invalid mode give. either CGAL  or CVX "<<endl;
        exit(-1);
    }
   /**
   std::cout<<"px: ";
    for(int n=0;n<n_seg;n++)
        std::cout<<spline_x.spline[n].transpose()<<" ";

    std::cout<<std::endl;
	**/

    // solve for y
    CVXGEN_OPTI::construct_qp(params,Q,H_wpnt_y,Aeq,beqy);
    solve();
    spline_y=CVXGEN_OPTI::get_solution(vars,poly_order,n_seg);
    
	/**
	std::cout<<"py: ";
    for(int n=0;n<n_seg;n++)
        std::cout<<spline_y.spline[n].transpose()<<" ";
    std::cout<<std::endl;

	**/
    
	// solve for z
    CVXGEN_OPTI::construct_qp(params,Q,H_wpnt_z,Aeq,beqz);
    solve();
    spline_z=CVXGEN_OPTI::get_solution(vars,poly_order,n_seg);
    
	/**
	std::cout<<"pz: ";
    for(int n=0;n<n_seg;n++)
        std::cout<<spline_z.spline[n].transpose()<<" ";
    std::cout<<std::endl;
	**/


    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();
//    std::cout<<"[CVXGEN] elapsed time: "<<dt<<std::endl;

    TrajGen::PolySplineXYZ splineXYZ;

    splineXYZ.pxs=spline_x;
    splineXYZ.pys=spline_y;
    splineXYZ.pzs=spline_z;
    splineXYZ.checkpnts=ts;

    return splineXYZ;
}


nav_msgs::Path TrajGen::horizon_eval_spline(const PolySplineXYZ& spline , int N_eval_interval) {


    geometry_msgs::PoseStamped poseStamped;
    nav_msgs::Path path;
    int n_seg=spline.checkpnts.size()-1;
    int poly_order=spline.pys.poly_order;


    // per each segment
    for(int n=0;n<n_seg;n++){
        // evaluation start and end
        TrajGen::Time t1=spline.checkpnts.coeff(n);
        TrajGen::Time t2=spline.checkpnts.coeff(n+1);

        // time horizon between start and end
        VectorXd eval_time_horizon(N_eval_interval);
        eval_time_horizon.setLinSpaced(N_eval_interval,t1,t2);

        // evaluation path on that horizon
        for(int t_idx=0;t_idx<N_eval_interval;t_idx++){
            Time t_eval=eval_time_horizon.coeff(t_idx)-t1;
            poseStamped.pose.position.x=t_vec(poly_order,t_eval,0).transpose()*spline.pxs.spline[n];
            poseStamped.pose.position.y=t_vec(poly_order,t_eval,0).transpose()*spline.pys.spline[n];
            poseStamped.pose.position.z=t_vec(poly_order,t_eval,0).transpose()*spline.pzs.spline[n];
            path.poses.push_back(poseStamped);

        }
    }

    return path;
}


geometry_msgs::Point TrajGen::point_eval_spline(const TrajGen::PolySplineXYZ & spline, TrajGen::Time t_eval) {


    geometry_msgs::Point eval_point;

    int poly_order=spline.pxs.poly_order;

    Eigen::Index spline_idx=TrajGen::find_spline_interval(spline.checkpnts,t_eval);

	if(spline_idx==-1){
		std::cerr<<"could not get the spline of t_eval: please exit "<<std::endl; 
        std::abort();
	}
	else{
    eval_point.x=t_vec(poly_order,t_eval-spline.checkpnts[spline_idx],0).transpose()*spline.pxs.spline[spline_idx];
    eval_point.y=t_vec(poly_order,t_eval-spline.checkpnts[spline_idx],0).transpose()*spline.pys.spline[spline_idx];
    eval_point.z=t_vec(poly_order,t_eval-spline.checkpnts[spline_idx],0).transpose()*spline.pzs.spline[spline_idx];
	}
    return eval_point;
}


 MatrixXd TrajGen::integral_jerk_squared(PolyOrder poly_order){
    // this is ingeral of jerk matrix from 0 to 1 given polynomial order
     int n=poly_order;
    MatrixXd Qj(n+1,n+1);
    Qj.setZero();

    for(int i=3;i<n+1;i++)
        for(int j=3;j<n+1;j++)
            if(i==3 and j==3)
                Qj.coeffRef(i,j)=i*(i-1)*(i-2)*j*(j-1)*(j-2);
            else
                Qj.coeffRef(i,j)=i*(i-1)*(i-2)*j*(j-1)*(j-2)/(i+j-5);


    return Qj;
}

MatrixXd TrajGen::time_scailing_mat(Time t1, Time t2, PolyOrder poly_order) {
    float d=t2-t1;
    MatrixXd D(poly_order+1,poly_order+1);
    D.setZero();

    for(int i=0;i<poly_order+1;i++)
        D.coeffRef(i,i)=pow(d,i);
    return D;
}


VectorXd TrajGen::t_vec(PolyOrder poly_order, Time t, N_diff n_diff) {
    VectorXd vec(poly_order+1);
    vec.setZero();
    switch(n_diff){
        case 0:
            for(int i=0;i<poly_order+1;i++)
                vec.coeffRef(i)=pow(t,i);
            break;
        case 1:
            for(int i=1;i<poly_order+1;i++)
                vec.coeffRef(i)=i*pow(t,i-1);

            break;
        case 2:
            for(int i=2;i<poly_order+1;i++)
                vec.coeffRef(i)=i*(i-1)*pow(t,i-2);
            break;
    }

    return vec;
}


Eigen::Index TrajGen::find_spline_interval(const TimeSeries& ts,TrajGen::Time t_eval) {

    Eigen::Index idx=-1;

    for(int i=0;i<ts.size()-1;i++)
        if(ts.coeff(i)<=t_eval && ts.coeff(i+1)>t_eval)
            idx=i;

    return idx;

    // if idx == -1, then could not find
}








