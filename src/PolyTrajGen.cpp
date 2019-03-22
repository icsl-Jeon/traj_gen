#include "traj_gen/PolyTrajGen.h"

// constructor 
PathPlanner::PathPlanner():is_path_computed(false){};

void PathPlanner::path_gen(const TimeSeries& knots ,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt ){

    int n_seg = waypoints.poses.size() - 1;
    int poly_order = opt.poly_order;
    QP_form_xyz qp_xyz = qp_gen(knots,waypoints,v0,a0,opt);
    bool is_ok_x,is_ok_y,is_ok_z;
    PolySpline spline_x = get_solution (solveqp(qp_xyz.x,is_ok_x),poly_order,n_seg);
    PolySpline spline_y = get_solution (solveqp(qp_xyz.y,is_ok_y),poly_order,n_seg);
    PolySpline spline_z = get_solution (solveqp(qp_xyz.z,is_ok_z),poly_order,n_seg);

    is_path_computed = true;

    // finalizing the path 
    spline_xyz.spline_x = spline_x;
    spline_xyz.spline_y = spline_y;
    spline_xyz.spline_z = spline_z;
    spline_xyz.is_valid = is_ok_x and is_ok_y and is_ok_z;
    spline_xyz.knot_time.assign(knots.data(),knots.data()+knots.size()) ;
    spline_xyz.n_seg = spline_x.n_seg;
}


QP_form_xyz PathPlanner::qp_gen(const TimeSeries& knots,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt ){



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
    if (opt.is_waypoint_soft)
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

    MatrixXd Aeq0(3,n_var_total),beq0_sub(3,1);
    Aeq0.block(0,0,3,poly_order+1)  = get_init_constraint_mat(waypoints.poses[0].pose.position.x,v0.linear.x,a0.linear.x,opt).A;
    row_append(Aeq_x,Aeq0); 
    row_append(Aeq_y,Aeq0); 
    row_append(Aeq_z,Aeq0);         

    beq0_sub.block(0,0,3,1)  = get_init_constraint_mat(waypoints.poses[0].pose.position.x,v0.linear.x,a0.linear.x,opt).b;     
    row_append(beq_x,beq0_sub);
    beq0_sub.block(0,0,3,1)  = get_init_constraint_mat(waypoints.poses[0].pose.position.y,v0.linear.y,a0.linear.y,opt).b;  
    row_append(beq_y,beq0_sub);
    beq0_sub.block(0,0,3,1)  = get_init_constraint_mat(waypoints.poses[0].pose.position.z,v0.linear.z,a0.linear.z,opt).b;     
    row_append(beq_z,beq0_sub);    
   
    // (2) Waypoints constraints (if it is hard constrained)

    if(not opt.is_waypoint_soft)
        for(int k = 0;k<n_seg;k++){            
            int insert_idx = k*blck_size;
            MatrixXd Dn = time_scailing_mat(knots[k + 1]-knots[k], poly_order);
            MatrixXd Aeq_sub(1,n_var_total),beq_sub(1,1);

            Aeq_sub.block(0,insert_idx,1,blck_size) = t_vec(poly_order,1,0).transpose()*Dn;
            row_append(Aeq_x,Aeq_sub); 
            row_append(Aeq_y,Aeq_sub); 
            row_append(Aeq_z,Aeq_sub);             
            
            beq_sub(0) = waypoints.poses[k].pose.position.x;
            row_append(beq_x,beq_sub);
            beq_sub(0) = waypoints.poses[k].pose.position.y;
            row_append(beq_y,beq_sub);
            beq_sub(0) = waypoints.poses[k].pose.position.z;
            row_append(beq_z,beq_sub);
            
        }


    // (3) continuity constraints 
    
    for(int k = 0;k<n_seg-1;k++){
        int insert_idx = k*blck_size;        
        MatrixXd Aeq_sub(3,n_var_total),beq_sub(3,1);
        Aeq_sub.block(0,insert_idx,3,2*blck_size) = get_continuity_constraint_mat(knots[k+1]-knots[k],knots[k+2]-knots[k+1],opt).A;
        beq_sub = get_continuity_constraint_mat(knots[k+1]-knots[k],knots[k+2]-knots[k+1],opt).b;
        row_append(Aeq_x,Aeq_sub); row_append(beq_x,beq_sub);
        row_append(Aeq_y,Aeq_sub); row_append(beq_y,beq_sub);
        row_append(Aeq_z,Aeq_sub); row_append(beq_z,beq_sub);              
    }


    /*
        3. Inequality constraints  
    */
    //  -----------------------------------------------------------------------    


    /*
        4. Wrapping  
    */
    //  -----------------------------------------------------------------------    

    QP_form qp_x; qp_x.Q = Qx; qp_x.H = Hx; qp_x.A = Aineq_x; qp_x.b = bineq_x; qp_x.Aeq = Aeq_x; qp_x.beq = beq_x; 
    QP_form qp_y; qp_y.Q = Qy; qp_y.H = Hy; qp_y.A = Aineq_y; qp_y.b = bineq_y; qp_y.Aeq = Aeq_y; qp_y.beq = beq_y; 
    QP_form qp_z; qp_z.Q = Qz; qp_z.H = Hz; qp_z.A = Aineq_z; qp_z.b = bineq_z; qp_z.Aeq = Aeq_z; qp_z.beq = beq_z; 
    
    QP_form_xyz qp_prob_xyz;
    qp_prob_xyz.x = qp_x;
    qp_prob_xyz.y = qp_y;
    qp_prob_xyz.z = qp_z;
    
    return qp_prob_xyz;
};


/*
    sub routines   
*/

//  -----------------------------------------------------------------------    


VectorXd PathPlanner::solveqp(QP_form qp_prob,bool& is_ok){
    is_ok = true;
    MatrixXd Q = qp_prob.Q;
    MatrixXd H = qp_prob.H;
    MatrixXd Aineq = qp_prob.A;
    MatrixXd bineq = qp_prob.b;
    MatrixXd Aeq = qp_prob.Aeq;
    MatrixXd beq = qp_prob.beq;
     
    USING_NAMESPACE_QPOASES;

    int N_var = Q.rows();
    int N_ineq_const = Aineq.rows();
    int N_eq_const = Aeq.rows();    
    int N_const = N_ineq_const + N_eq_const;

    real_t H_qp[N_var*N_var];
    real_t g[N_var];
    real_t A[N_var*N_const];
    real_t lbA[N_const];
    real_t ubA[N_const];

    // cost
    for (int i = 0;i<N_var;i++){
        g[i] = H(0,i);            
        for(int j = 0;j<N_var;j++)
            H_qp[j*N_var+i] = 2*Q(i,j);
    }
    
    // eq constraints
    for (int r = 0; r<N_eq_const;r++){
        lbA[r] = beq(r);
        ubA[r] = beq(r);
        for(int c= 0; c<N_var;c++)
            A[r*N_var + c] = Aeq(r,c);
    }

    // ineq constraints
    for (int r = 0; r<N_ineq_const;r++){
        lbA[N_eq_const+r] = -std::numeric_limits<double>::min();
        ubA[N_eq_const+r] = bineq(r);
        for(int c= 0; c<N_var;c++)
            A[(r+N_eq_const)*N_var + c] = Aineq(r,c);
    }    

    int_t nWSR = 2000;
   
	QProblem qp_obj(N_var,N_const);
    Options options;
	options.printLevel = PL_HIGH;
	qp_obj.setOptions(options);
	qp_obj.init(H_qp,g,A,NULL,NULL,lbA,ubA,nWSR);
    if(qp_obj.isInfeasible()){
        cout<<"[QP solver] warning: problem is infeasible. "<<endl;
        is_ok = false;
    }
    real_t xOpt[N_var];
    qp_obj.getPrimalSolution(xOpt);

    if(qp_obj.isSolved()){
        cout<<"[QP solver] warning: optimization terminated immaturely."<<endl;
        is_ok = false;
    }
    
    VectorXf sol(N_var);

    for(int n = 0; n<N_var;n++)
        sol(n) = xOpt[n];

    return sol;        
}


PolySpline PathPlanner::get_solution(VectorXf sol,int poly_order,int n_seg ){
            
    // this initialization of spline object 
    PolySpline polySpline;
    
    polySpline.n_seg=n_seg;
    // polySpline.knot_time.reserve(n_seg+1);  // let's insert the time later outside of this fucntion 
    polySpline.poly_coeff.reserve(n_seg);  // let's insert the time later outside of this fucntion 
        
    for(uint i = 0; i<polySpline.n_seg;i++)
        polySpline.poly_coeff[i].poly_order=poly_order;
    int n_var=n_seg*(poly_order+1);

    // std::cout<<"[DEBUG] solution:"<<std::endl;
    // std::cout<<var<<std::endl;
    // from lowest order 0
    for(int n=0;n<n_seg;n++){
        PolyCoeff coeff;
        coeff.coeff.resize(poly_order+1);
        for(int i=0;i<poly_order+1;i++){
            coeff.coeff[i]=sol(n*(poly_order+1)+i); 
            coeff.poly_order = poly_order;         
        }
        polySpline.poly_coeff.push_back(coeff);
    }

    return polySpline;

}



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

// output size 3x(2(N+1))
Constraint PathPlanner::get_continuity_constraint_mat(double dt1,double dt2,TrajGenOpts opt){
    
    int poly_order = opt.poly_order;
    MatrixXd Aeq(3,2*(poly_order+1));
    MatrixXd beq(3,1); beq.setZero();   
    MatrixXd D1 = time_scailing_mat(dt1,poly_order);
    MatrixXd D2 = time_scailing_mat(dt2,poly_order);

    // 0th order         
    Aeq.block(0,0,1,poly_order+1) = t_vec(poly_order,1,0).transpose()*D1;
    Aeq.block(0,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,0).transpose()*D2;

    // 1st order
    Aeq.block(1,0,1,poly_order+1) = t_vec(poly_order,1,1).transpose()*D1/dt1;
    Aeq.block(1,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,1).transpose()*D2/dt2; 

    // 2nd order
    Aeq.block(2,0,1,poly_order+1) = t_vec(poly_order,1,2).transpose()*D1/pow(dt1,2);
    Aeq.block(2,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,2).transpose()*D2/pow(dt2,2); 
           
    Constraint constraint;
    constraint.A = Aeq;
    constraint.b = beq;
    
    return constraint;
}





void row_append(MatrixXd & mat,MatrixXd mat_sub){
    mat.conservativeResize(mat.rows()+mat_sub.rows(),mat.cols());
    mat.block(mat.rows(),0,mat_sub.rows(),mat.cols()) = mat_sub;
}
