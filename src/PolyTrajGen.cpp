#include "traj_gen/PolyTrajGen.h"

// constructor 
PathPlanner::PathPlanner():is_path_computed(false){};

void PathPlanner::path_gen(const TimeSeries& knots ,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt ){
    
    is_this_verbose = opt.verbose;
    int n_seg = waypoints.poses.size() - 1;
    int poly_order = opt.poly_order;
    QP_form_xyz qp_xyz = qp_gen(knots,waypoints,v0,a0,opt);
    bool is_ok_x,is_ok_y,is_ok_z;

    PolySpline spline_x = get_solution (solveqp(qp_xyz.x,is_ok_x),poly_order,n_seg);
    PolySpline spline_y = get_solution (solveqp(qp_xyz.y,is_ok_y),poly_order,n_seg);
    PolySpline spline_z = get_solution (solveqp(qp_xyz.z,is_ok_z),poly_order,n_seg);

        // interpret the solution and rescaling
        cout<<"------------solution in increasing order---------------"<<endl;
        for(int k=0;k<n_seg;k++){
            cout<<"segment "<<k+1<<endl;
            cout<<"x: ";
            for(int n = 0; n<=poly_order ; n++){
                spline_x.poly_coeff[k].coeff[n] /= pow(knots[k+1]-knots[k],n);
                printf("%.4f , ",spline_x.poly_coeff[k].coeff[n]);
            }
            cout<<endl;
            cout<<"y: ";
            for(int n = 0; n<=poly_order ; n++){
                spline_y.poly_coeff[k].coeff[n] /= pow(knots[k+1]-knots[k],n);
                printf("%.4f , ",spline_y.poly_coeff[k].coeff[n]);
            }
            cout<<endl;
            cout<<"z: ";
            for(int n = 0; n<=poly_order ; n++){
                spline_z.poly_coeff[k].coeff[n] /= pow(knots[k+1]-knots[k],n);
                printf("%.4f , ",spline_z.poly_coeff[k].coeff[n]);
            }
            cout<<endl;
        }
    
    is_path_computed = true;

    // finalizing the path 
    spline_xyz.spline_x = spline_x;
    spline_xyz.spline_y = spline_y;
    spline_xyz.spline_z = spline_z;
    spline_xyz.is_valid = is_ok_x and is_ok_y and is_ok_z;
    spline_xyz.knot_time.assign(knots.data(),knots.data()+knots.size()) ;
    spline_xyz.n_seg = spline_x.n_seg;
    spline_xyz.poly_order = poly_order;

    // update current path 
    horizon_eval_spline(10);

}


QP_form_xyz PathPlanner::qp_gen(const TimeSeries& knots,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt ){



    int n_seg = waypoints.poses.size() - 1;
    int poly_order = opt.poly_order;
    // this should be changed when we adopt single box 
    int n_var_total = (poly_order + 1) * n_seg; // in case of the seperable x,y,z optimization 
    int blck_size=poly_order+1;

    MatrixXd Q(n_var_total,n_var_total),H(1,n_var_total);
    Q.setZero(); H.setZero();
    MatrixXd Qx = Q, Qy = Q, Qz = Q; 
    MatrixXd Hx = H, Hy = H, Hz = H;

    MatrixXd Aeq(0,n_var_total),beq(0,1),Aineq(0,n_var_total),bineq(0,1);
    Aeq.setZero(); beq.setZero(); Aineq.setZero(); bineq.setZero();
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
            // Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=Dn*integral_jerk_squared(poly_order)*Dn/pow(dn,5);
            Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=integral_jerk_squared(poly_order);
        }         
    }
    // if minimum snap 
    else if(opt.objective_derivative == 4){
        for (int n = 0; n < n_seg; n++) {
            MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
            double dn = knots[n + 1] - knots[n];
            // Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=Dn*integral_snap_squared(poly_order)*Dn/pow(dn,7);
            Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=integral_snap_squared(poly_order);

        }
    }else{
        cerr<<"undefined derivative in objective"<<endl;
        return QP_form_xyz();
    }

    // if it is soft, we include the deviation term 
    if (opt.is_waypoint_soft){    
            for(int n=0;n<n_seg;n++){
                MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
                int insert_start=blck_size*(n);
                double time_scaling_factor;
                if (opt.objective_derivative == 3)
                    time_scaling_factor = pow(knots[n+1]-knots[n],5);
                else 
                    time_scaling_factor = pow(knots[n+1]-knots[n],7);

                cout<<"time scaling factor: "<<time_scaling_factor<<endl;
                Q.block(insert_start,insert_start,blck_size,blck_size)+=time_scaling_factor*opt.w_d*t_vec(poly_order,1,0)*t_vec(poly_order,1,0).transpose();
                Hx.block(0,insert_start,1,blck_size)=-2*time_scaling_factor*opt.w_d*(waypoints.poses[n+1].pose.position.x)*t_vec(poly_order,1,0).transpose();
                Hy.block(0,insert_start,1,blck_size)=-2*time_scaling_factor*opt.w_d*(waypoints.poses[n+1].pose.position.y)*t_vec(poly_order,1,0).transpose();
                Hz.block(0,insert_start,1,blck_size)=-2*time_scaling_factor*opt.w_d*(waypoints.poses[n+1].pose.position.z)*t_vec(poly_order,1,0).transpose();
            }
    
    }
    Qx = Q; Qy = Q; Qz = Q;



    /*
        2. Equality constraints  
    */
    //  -----------------------------------------------------------------------

    // (1) Initial constraints 

    MatrixXd Aeq0(3,n_var_total),beq0_sub(3,1);
    Aeq0.setZero(), beq0_sub.setZero();
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
            Aeq_sub.setZero(); beq_sub.setZero();
            Aeq_sub.block(0,insert_idx,1,blck_size) = t_vec(poly_order,1,0).transpose();
            row_append(Aeq_x,Aeq_sub); 
            row_append(Aeq_y,Aeq_sub); 
            row_append(Aeq_z,Aeq_sub);             
            
            beq_sub(0) = waypoints.poses[k+1].pose.position.x;
            row_append(beq_x,beq_sub);
            beq_sub(0) = waypoints.poses[k+1].pose.position.y;
            row_append(beq_y,beq_sub);
            beq_sub(0) = waypoints.poses[k+1].pose.position.z;
            row_append(beq_z,beq_sub);
            
        }


    // (3) continuity constraints 
    
    // if the objective is snap, we will include 3rd order continuity 
    for(int k = 0;k<n_seg-1;k++){
        int insert_idx = k*blck_size;        
        MatrixXd Aeq_sub(opt.objective_derivative,n_var_total),beq_sub(opt.objective_derivative,1);
        Aeq_sub.setZero(); beq_sub.setZero();
        Aeq_sub.block(0,insert_idx,opt.objective_derivative,2*blck_size) = get_continuity_constraint_mat(knots[k+1]-knots[k],knots[k+2]-knots[k+1],opt).A;
        beq_sub = get_continuity_constraint_mat(knots[k+1]-knots[k],knots[k+2]-knots[k+1],opt).b;
        row_append(Aeq_x,Aeq_sub); row_append(beq_x,beq_sub);
        row_append(Aeq_y,Aeq_sub); row_append(beq_y,beq_sub);
        row_append(Aeq_z,Aeq_sub); row_append(beq_z,beq_sub);              
    }


    /*
        3. Inequality constraints  
    */
    //  -----------------------------------------------------------------------    

    if(opt.is_multi_corridor){

        double safe_r = opt.safe_r;
        int N_safe_pnts = opt.N_safe_pnts;
        int n_ineq_consts = 2*(N_safe_pnts)*n_seg;
        int poly_order = opt.poly_order;

        safe_corridor_marker.header.frame_id = "/world";
        safe_corridor_marker.ns = "sf_corridor";
        safe_corridor_marker.type = 6; // cube list 
        safe_corridor_marker.scale.x = 2*safe_r;
        safe_corridor_marker.scale.y = 2*safe_r;
        safe_corridor_marker.scale.z = 2*safe_r;
        safe_corridor_marker.action = 0;
        safe_corridor_marker.pose.orientation.w = 1;
        safe_corridor_marker.color.a = 0.5;
        safe_corridor_marker.color.r = 170.0/255.0;
        safe_corridor_marker.color.g = 1.0;
        safe_corridor_marker.color.b = 1.0;
        safe_corridor_marker.points.clear();

        safe_corridor_marker.points.resize((N_safe_pnts)*n_seg);


        MatrixXd A_sub(n_ineq_consts,n_var_total),bx_sub(n_ineq_consts,1),by_sub(n_ineq_consts,1),bz_sub(n_ineq_consts,1);
        A_sub.setZero(); bx_sub.setZero(); by_sub.setZero(); bz_sub.setZero();
        
        int ineq_col_insert_idx1=0,ineq_row_insert_idx = 0; 
        int idx = 0;

        for (int n = 0 ; n<n_seg;n++){

            // sub points along this line segment 
            double x0,y0,z0;
            double xf,yf,zf;
            double dx,dy,dz;
            
            x0 = waypoints.poses[n].pose.position.x;
            y0 = waypoints.poses[n].pose.position.y;
            z0 = waypoints.poses[n].pose.position.z;

            xf = waypoints.poses[n+1].pose.position.x;
            yf = waypoints.poses[n+1].pose.position.y;
            zf = waypoints.poses[n+1].pose.position.z;
            
            dx = (xf-x0)/((N_safe_pnts+1));
            dy = (yf-y0)/((N_safe_pnts+1));
            dz = (zf-z0)/((N_safe_pnts+1));
            for (int n_sub = 1; n_sub<=N_safe_pnts;n_sub++){

                double x_sub,y_sub,z_sub;
                x_sub = x0 + dx*n_sub;
                y_sub = y0 + dy*n_sub;
                z_sub = z0 + dz*n_sub;
                
                safe_corridor_marker.points[idx].x = x_sub;
                safe_corridor_marker.points[idx].y = y_sub;
                safe_corridor_marker.points[idx].z = z_sub;
                idx++;

                double t_control = 1.0/(N_safe_pnts+1) * n_sub;

                // lower limit 
                A_sub.block(ineq_row_insert_idx,ineq_col_insert_idx1,1,blck_size)=-t_vec(poly_order,t_control,0).transpose();        
                bx_sub.coeffRef(ineq_row_insert_idx) = -(x_sub - safe_r);
                by_sub.coeffRef(ineq_row_insert_idx) = -(y_sub - safe_r);
                bz_sub.coeffRef(ineq_row_insert_idx) = -(z_sub - safe_r);        

                ineq_row_insert_idx++;

                // upper limit 
                A_sub.block(ineq_row_insert_idx,ineq_col_insert_idx1,1,blck_size)=t_vec(poly_order,t_control,0).transpose();        
                bx_sub.coeffRef(ineq_row_insert_idx) = (x_sub + safe_r);
                by_sub.coeffRef(ineq_row_insert_idx) = (y_sub + safe_r);
                bz_sub.coeffRef(ineq_row_insert_idx) = (z_sub + safe_r);      

                ineq_row_insert_idx++;

            }
            ineq_col_insert_idx1 += blck_size;
        }

        row_append(Aineq_x,A_sub); row_append(bineq_x,bx_sub);
        row_append(Aineq_y,A_sub); row_append(bineq_y,by_sub);
        row_append(Aineq_z,A_sub); row_append(bineq_z,bz_sub); 

    }


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

void PathPlanner::horizon_eval_spline(int N_eval_interval){

    geometry_msgs::PoseStamped poseStamped;
    nav_msgs::Path path; 

    ROS_INFO("evaluating spline....");

    int n_seg=spline_xyz.n_seg;
    int poly_order = spline_xyz.poly_order;
    
    printf("n_seg : %d, poly_order: %d, \n",spline_xyz.n_seg,spline_xyz.poly_order);

    // std::cout<<spline.knot_time.size()<<std::endl;
    // per each segment
    for(int n=0;n<n_seg;n++){
        // evaluation start and end
        
        double t1=spline_xyz.knot_time[n];
        double t2=spline_xyz.knot_time[n+1];

        // time horizon between start and end
        VectorXd eval_time_horizon(N_eval_interval);
        eval_time_horizon.setLinSpaced(N_eval_interval,t1,t2);

        // evaluation path on that horizon
        for(int t_idx=0;t_idx<N_eval_interval;t_idx++){
            // double t_eval=(eval_time_horizon.coeff(t_idx)-t1)/(t2-t1);
            double t_eval=(eval_time_horizon.coeff(t_idx)-t1);

            poseStamped.pose.position.x=t_vec(poly_order,t_eval,0).transpose()*Map<VectorXd>(spline_xyz.spline_x.poly_coeff[n].coeff.data(),poly_order+1);
            poseStamped.pose.position.y=t_vec(poly_order,t_eval,0).transpose()*Map<VectorXd>(spline_xyz.spline_y.poly_coeff[n].coeff.data(),poly_order+1);
            poseStamped.pose.position.z=t_vec(poly_order,t_eval,0).transpose()*Map<VectorXd>(spline_xyz.spline_z.poly_coeff[n].coeff.data(),poly_order+1);
            path.poses.push_back(poseStamped);
        }
    }

     current_path=path;

}


geometry_msgs::Point PathPlanner::point_eval_spline(double t_eval) {


    geometry_msgs::Point eval_point;

    int poly_order=spline_xyz.poly_order;
	// DEBUG
//			std::cout<<"knot time of this: "<<std::endl;
	for(auto it = spline_xyz.knot_time.begin();it<spline_xyz.knot_time.end();it++)
//		std::cout<<*it<<", ";spline_xyz
//	std::cout<<std::endl;
//    std::cout<<"point_eval: "<<t_eval.toSec()<<"knot time final: "<<spline.knot_time.back()<<std::endl;
    t_eval =min(spline_xyz.knot_time.back(),t_eval);
    t_eval =max(spline_xyz.knot_time.front(),t_eval);

    Eigen::Index spline_idx=find_spline_interval(spline_xyz.knot_time,t_eval);
//	std::cout<<"Index: "<<spline_idx<<std::endl;
    // double t_eval_norm = (t_eval-spline_xyz.knot_time[spline_idx])/(spline_xyz.knot_time[spline_idx+1]-spline_xyz.knot_time[spline_idx]);
    double t_eval_norm = (t_eval-spline_xyz.knot_time[spline_idx]);

    eval_point.x=t_vec(poly_order,t_eval_norm,0).transpose()*Map<VectorXd>(spline_xyz.spline_x.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_point.y=t_vec(poly_order,t_eval_norm,0).transpose()*Map<VectorXd>(spline_xyz.spline_y.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_point.z=t_vec(poly_order,t_eval_norm,0).transpose()*Map<VectorXd>(spline_xyz.spline_z.poly_coeff[spline_idx].coeff.data(),poly_order+1);
	
    return eval_point;
}

geometry_msgs::Twist PathPlanner::vel_eval_spline(double t_eval){

    geometry_msgs::Twist eval_vel;
    int poly_order=spline_xyz.poly_order;
    t_eval =min(spline_xyz.knot_time.back(),t_eval);
    Eigen::Index spline_idx=find_spline_interval(spline_xyz.knot_time,t_eval);  
    double t_eval_norm = (t_eval-spline_xyz.knot_time[spline_idx]);

    eval_vel.linear.x=t_vec(poly_order,t_eval_norm,1).transpose()*Map<VectorXd>(spline_xyz.spline_x.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_vel.linear.y=t_vec(poly_order,t_eval_norm,1).transpose()*Map<VectorXd>(spline_xyz.spline_y.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_vel.linear.z=t_vec(poly_order,t_eval_norm,1).transpose()*Map<VectorXd>(spline_xyz.spline_z.poly_coeff[spline_idx].coeff.data(),poly_order+1);
	
    return eval_vel;

}

geometry_msgs::Twist PathPlanner::accel_eval_spline(double t_eval){

    geometry_msgs::Twist eval_acc;
    int poly_order=spline_xyz.poly_order;
    t_eval =min(spline_xyz.knot_time.back(),t_eval);
    Eigen::Index spline_idx=find_spline_interval(spline_xyz.knot_time,t_eval);  
    double t_eval_norm = (t_eval-spline_xyz.knot_time[spline_idx]);

    eval_acc.linear.x=t_vec(poly_order,t_eval_norm,2).transpose()*Map<VectorXd>(spline_xyz.spline_x.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_acc.linear.y=t_vec(poly_order,t_eval_norm,2).transpose()*Map<VectorXd>(spline_xyz.spline_y.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_acc.linear.z=t_vec(poly_order,t_eval_norm,2).transpose()*Map<VectorXd>(spline_xyz.spline_z.poly_coeff[spline_idx].coeff.data(),poly_order+1);
	
    return eval_acc;

}



VectorXd PathPlanner::solveqp(QP_form qp_prob,bool& is_ok){
    is_ok = true;
    MatrixXd Q = qp_prob.Q;
    MatrixXd H = qp_prob.H;
    MatrixXd Aineq = qp_prob.A;
    MatrixXd bineq = qp_prob.b;
    MatrixXd Aeq = qp_prob.Aeq;
    MatrixXd beq = qp_prob.beq;
    if(is_this_verbose){
        cout<<"Q: "<<endl;
        cout<<Q<<endl;
        
        cout<<"H: "<<endl;
        cout<<H<<endl;
            
        cout<<"Aineq: "<<endl;
        cout<<Aineq<<endl;        

        cout<<"bineq: "<<endl;
        cout<<bineq<<endl;  

        cout<<"Aeq: "<<endl;
        cout<<Aeq<<endl;        

        cout<<"beq: "<<endl;
        cout<<beq<<endl;  
    }
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
        lbA[N_eq_const+r] = -1e+8;
        ubA[N_eq_const+r] = bineq(r);
        for(int c= 0; c<N_var;c++)
            A[(r+N_eq_const)*N_var + c] = Aineq(r,c);
    }    

    int_t nWSR = 2000;
   
	QProblem qp_obj(N_var,N_const,HST_SEMIDEF);
    std::cout<<"hessian type: "<<qp_obj.getHessianType()<<endl;
    Options options;
	options.printLevel = PL_LOW;
	qp_obj.setOptions(options);
	qp_obj.init(H_qp,g,A,NULL,NULL,lbA,ubA,nWSR);
    if(qp_obj.isInfeasible()){
        cout<<"[QP solver] warning: problem is infeasible. "<<endl;
        is_ok = false;
    }
    real_t xOpt[N_var];
    qp_obj.getPrimalSolution(xOpt);

    if(not qp_obj.isSolved()){
        cout<<"[QP solver] quadratic programming has not been solved "<<endl;
        is_ok = false;
    }
    
    VectorXd sol(N_var);

    for(int n = 0; n<N_var;n++)
        sol(n) = xOpt[n];
    return sol;        
}


PolySpline PathPlanner::get_solution(VectorXd sol,int poly_order,int n_seg ){
            
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
    Aeq0.setZero();
    beq0.setZero();
    double dt_arbitrary = 1;

    // Aeq0.row(0) = t_vec(poly_order,0,0).transpose()*time_scailing_mat(dt_arbitrary,poly_order);
    // Aeq0.row(1) = t_vec(poly_order,0,1).transpose()*time_scailing_mat(dt_arbitrary,poly_order)/dt_arbitrary;
    // Aeq0.row(2) = t_vec(poly_order,0,2).transpose()*time_scailing_mat(dt_arbitrary,poly_order)/pow(dt_arbitrary,2);

    Aeq0.row(0) = t_vec(poly_order,0,0).transpose();
    Aeq0.row(1) = t_vec(poly_order,0,1).transpose()/dt_arbitrary;
    Aeq0.row(2) = t_vec(poly_order,0,2).transpose()/pow(dt_arbitrary,2);

    beq0(0) = x0;
    beq0(1) = v0;
    beq0(2) = a0;
    
    Constraint constraint;
    constraint.A = Aeq0;
    constraint.b = beq0;
    return constraint;
}


visualization_msgs::Marker PathPlanner::get_knots_marker(){
 
    visualization_msgs::Marker knots_marker;

    knots_marker.ns = "knots";
    knots_marker.id = 0;
    knots_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    knots_marker.color.r = 10.0/255.0;
    knots_marker.color.g = 50.0/255.0;
    knots_marker.color.b = 1.0;
    knots_marker.color.a = 0.8;
    knots_marker.pose.orientation.w = 1.0;
    double scale = 0.2; 
    knots_marker.scale.x = scale;
    knots_marker.scale.y = scale;
    knots_marker.scale.z = scale;       
    
    for (auto it = spline_xyz.knot_time.begin(); it<spline_xyz.knot_time.end();it++)
        knots_marker.points.push_back(point_eval_spline(*it));

    return knots_marker;
}

// output size 3x(2(N+1))
Constraint PathPlanner::get_continuity_constraint_mat(double dt1,double dt2,TrajGenOpts opt){
    
    // int N_constraint = opt.objective_derivative;
    int N_constraint = 4;
    int poly_order = opt.poly_order;
    MatrixXd Aeq(N_constraint,2*(poly_order+1));
    MatrixXd beq(N_constraint,1); beq.setZero();   
    Aeq.setZero();
    beq.setZero();
    MatrixXd D1 = time_scailing_mat(dt1,poly_order);
    MatrixXd D2 = time_scailing_mat(dt2,poly_order);

    // 0th order         
    // Aeq.block(0,0,1,poly_order+1) = t_vec(poly_order,1,0).transpose()*D1;
    // Aeq.block(0,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,0).transpose()*D2;

    Aeq.block(0,0,1,poly_order+1) = t_vec(poly_order,1,0).transpose();
    Aeq.block(0,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,0).transpose();


    // 1st order
    // Aeq.block(1,0,1,poly_order+1) = t_vec(poly_order,1,1).transpose()*D1*dt2;
    // Aeq.block(1,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,1).transpose()*D2*dt1; 

    Aeq.block(1,0,1,poly_order+1) = t_vec(poly_order,1,1).transpose()*dt2;
    Aeq.block(1,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,1).transpose()*dt1; 


    // 2nd order
    Aeq.block(2,0,1,poly_order+1) = t_vec(poly_order,1,2).transpose()*pow(dt2,2);
    Aeq.block(2,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,2).transpose()*pow(dt1,2); 

    // if (N_constraint == 4){    
        // 3rd order
        Aeq.block(3,0,1,poly_order+1) = t_vec(poly_order,1,3).transpose()*pow(dt2,3);
        Aeq.block(3,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,3).transpose()*pow(dt1,3); 
    // }

    Constraint constraint;
    constraint.A = Aeq;
    constraint.b = beq;
    
    return constraint;
}




MatrixXd time_scailing_mat(double dt, int poly_order) {
    MatrixXd D(poly_order+1,poly_order+1);
    D.setZero();

    for(int i=0;i<poly_order+1;i++)
        D.coeffRef(i,i)=pow(dt,i);
    return D;
}


VectorXd t_vec(int poly_order, double t, int n_diff) {
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

        case 3:
            for(int i=3;i<poly_order+1;i++)
                vec.coeffRef(i)=i*(i-1)*(i-2)*pow(t,i-3);
            break;
    }

    return vec;
}


 MatrixXd integral_jerk_squared(int poly_order){
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

 MatrixXd integral_snap_squared(int poly_order){
    // this is ingeral of jerk matrix from 0 to 1 given polynomial order
     int n=poly_order;
    MatrixXd Qj(n+1,n+1);
    Qj.setZero();

    for(int i=4;i<n+1;i++)
        for(int j=4;j<n+1;j++)
            if(i==4 and j==4)
                Qj.coeffRef(i,j)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3);
            else
                Qj.coeffRef(i,j)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-7);


    return Qj;
}

void row_append(MatrixXd & mat,MatrixXd mat_sub){
    int orig_row_size = mat.rows();
    mat.conservativeResize(mat.rows()+mat_sub.rows(),mat.cols());
    mat.block(orig_row_size,0,mat_sub.rows(),mat.cols()) = mat_sub;

}



int find_spline_interval(const vector<double>& ts,double t_eval) {


    int idx=-1;
    
    for(int i=0;i<ts.size()-1;i++)
        if(ts[i]<=t_eval && ts[i+1]>t_eval)
            idx=i;
    if (t_eval >= ts.back())
        idx = ts.size()-2;

    return idx;

    // if idx == -1, then could not find
}
