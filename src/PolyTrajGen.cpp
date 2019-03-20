#include "traj_gen/PolyTrajGen.h"

PathPlanner::PathPlanner():is_path_computed(false){

};

// output size 3x(N+1)
Constraint PathPlanner::get_init_constraint_mat(double x0,double v0,double a0,TrajGenOpts opt){
    int poly_order = opt.poly_order;
    MatrixXd Aeq0(3,poly_order+1);
    MatrixXd beq0(3,1);
    double dt_arbitrary = 1;

    Aeq0.row(0) = t_vec(poly_order,0,0).transpose()*time_scailing_mat(dt_arbitrary,poly_order);
    Aeq0.row(1) = t_vec(poly_order,0,1).transpose()*time_scailing_mat(dt_arbitrary,poly_order)/dt_arbitrary;
    Aeq0.row(2) = t_vec(poly_order,0,2).transpose()*time_scailing_mat(dt_arbitrary,poly_order)/pow(dt_arbitrary,2);
    beq0(0) = x0;
    beq0(1) = v0;
    beq0(2) = a0;
    
    Constraint constraint;
    constraint.A = Aeq0;
    constraint.b = beq0;
    return constraint;
}

void PathPlanner::path_gen(const TimeSeries& knots,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt ){



    int n_seg = waypoints.poses.size() - 1;
    int poly_order = opt.poly_order;
    // this should be changed when we adopt single box 
    int n_var_total = (poly_order + 1) * n_seg; // in case of the seperable x,y,z optimization 
    int blck_size=poly_order+1;

    MatrixXd Q(n_var_total,n_var_total),H(1,n_var_total);
    MatrixXd Qx = Q, Qy = Q, Qz = Q; 
    MatrixXd Hx = H, Hy = H, Hz = H;

    MatrixXd Aeq(0,n_var_total),beq(0,n_var_total),Aineq(0,n_var_total),bineq(0,n_var_total);
    MatrixXd Aeq_x = Aeq, Aeq_y = Aeq, Aeq_z =Aeq;
    MatrixXd beq_x = beq, beq_y = beq, beq_z =beq;  
    MatrixXd Aineq_x = Aineq, Aineq_y = Aineq, Aineq_z =Aineq;
    MatrixXd bineq_x = bineq, bineq_y = bineq, bineq_z =bineq;  
    

    /*
        1. Cost funtion 
    */
    //  -----------------------------------------------------------------------

    // if minimum jerk 
    if (opt.objective_derivative == 3){    
        for (int n = 0; n < n_seg; n++) {
            MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
            double dn = knots[n + 1] - knots[n];
            Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=Dn*integral_jerk_squared(poly_order)*Dn/pow(dn,5);
        }         
    }
    // if minimum snap 
    else if(opt.objective_derivative == 4){
        for (int n = 0; n < n_seg; n++) {
            MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
            double dn = knots[n + 1] - knots[n];
            Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=Dn*integral_snap_squared(poly_order)*Dn/pow(dn,7);
        }
    }else{
        cerr<<"undefined derivative in objective"<<endl;
        return;
    }

    // if it is soft, we include the deviation term 
    for(int n=0;n<n_seg;n++){
        MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
        int insert_start=blck_size*(n);
        Q.block(insert_start,insert_start,blck_size,blck_size)+=opt.w_d*Dn*t_vec(poly_order,1,0)*t_vec(poly_order,1,0).transpose()*Dn;
        Hx.block(0,insert_start,1,blck_size)=-2*(waypoints.poses[n+1].pose.position.x)*t_vec(poly_order,1,0).transpose()*Dn;
        Hy.block(0,insert_start,1,blck_size)=-2*(waypoints.poses[n+1].pose.position.y)*t_vec(poly_order,1,0).transpose()*Dn;
        Hz.block(0,insert_start,1,blck_size)=-2*(waypoints.poses[n+1].pose.position.z)*t_vec(poly_order,1,0).transpose()*Dn;
    }

    Qx = Q; Qy = Q; Qz = Q;



    /*
        2. Equality constraints  
    */
    //  -----------------------------------------------------------------------

   // (1) Initial constraints 
   









    


};



void row_append(MatrixXd & mat,MatrixXd mat_sub){
    mat.conservativeResize(mat.rows()+1,mat.cols());
    mat.block()

    
}
