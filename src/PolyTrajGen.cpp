#include "traj_gen/PolyTrajGen.h"

// constructor 
PathPlanner::PathPlanner():is_path_computed(false){};


// R world to b (Rwb)
Affine3d PathPlanner::get_affine_corridor_pose(Point p1,Point p2){

    Affine3d Twb;

    // 1. rotation matrix 

        // e1 : x-axis of body axis attached to the centroid of 3D rectangle 
        Vector3d e1(p2.x - p1.x, p2.y - p1.y , p2.z - p1.z );     
        float box_l = e1.norm();
        e1.normalize();
        
        // e2 : y-axis
        Vector3d e2;
        e2(2) = 0; e2(1) = 1; // for simplicity 

		
		if (e1(0) != 0 and e1(1) != 0)   
		    e2(0) = - e1(1)/e1(0) * e2(1);
        else if (e1(0) == 0 and e1(1) != 0){
            e2(1) = 0, e2(0) = 1;
        }
        else if (e1(0) != 0 and e1(1) == 0){
            e2(0) = 0; e2(1) = 1;
        }
        else{
            e2(0) = e2(1) = 1/sqrt(2);
        }
		e2.normalize();
         
        // e3 : z-axis 
        Vector3d e3 = e1.cross(e2);
        
        Matrix3d Rwb; 
        Rwb.block(0,0,3,1) = e1; Rwb.block(0,1,3,1) = e2; Rwb.block(0,2,3,1) = e3;
        Vector3d pnt1(p1.x,p1.y,p1.z),pnt2(p2.x,p2.y,p2.z);  
    

        Affine3d rot(Rwb); 
        Affine3d trans(Translation3d((pnt1 + pnt2)/2));

        Twb = trans * rot;

        /**     
        cout<<"e1: "<<endl;
        cout<<e1<<endl;
        cout<<"e2: "<<endl;
        cout<<e2<<endl;
        cout<<"e3: "<<endl;
        cout<<e3<<endl;
        cout<<Rwb<<endl;
        cout << Twb.rotation() <<endl;
        cout << Twb.translation() <<endl;
        **/


    return Twb; 
}


// path generation core rountine 
void PathPlanner::path_gen(const TimeSeries& knots ,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt ){
    
    is_this_verbose = opt.verbose;
    int n_seg = waypoints.poses.size() - 1;
    int poly_order = opt.poly_order;

    PolySpline spline_x;
    PolySpline spline_y;
    PolySpline spline_z;
    bool is_ok= false; 
    // coupled 
    if(opt.is_single_corridor){
        // cout << knots <<endl;
        cout << "[TRAJ_GEN] xyz coupled corridor generation..." <<endl;
        QP_form qp;
        qp_gen(knots,waypoints,v0,a0,opt,&qp);
        PolySplineXYZ spline_xyz_sol = get_solution_couple(solveqp(qp,is_ok),poly_order,n_seg);
        spline_x = spline_xyz_sol.spline_x;        
        spline_y = spline_xyz_sol.spline_y; 
        spline_z = spline_xyz_sol.spline_z; 
         
    }else{ // decoupled 
        QP_form_xyz qp_xyz;
        cout << "[TRAJ_GEN] xyz decoupled corridor generation..." <<endl;
        qp_gen(knots,waypoints,v0,a0,opt,&qp_xyz);
        bool is_ok_x,is_ok_y,is_ok_z;            
        spline_x = get_solution (solveqp(qp_xyz.x,is_ok_x),poly_order,n_seg);
        spline_y = get_solution (solveqp(qp_xyz.y,is_ok_y),poly_order,n_seg);
        spline_z = get_solution (solveqp(qp_xyz.z,is_ok_z),poly_order,n_seg);
        is_ok  = is_ok_x and is_ok_y and is_ok_z;

    }


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
    spline_xyz.is_valid = is_ok;
    spline_xyz.knot_time.assign(knots.data(),knots.data()+knots.size()) ;
    spline_xyz.n_seg = spline_x.n_seg;
    spline_xyz.poly_order = poly_order;

    // update current path 
    horizon_eval_spline(10);
    write_spline(log_output_file_name);
}
/**
 * @brief This is qp generator for coupled case (single corridor)
 * 
 * @param knots 
 * @param waypoints 
 * @param v0 
 * @param a0 
 * @param opt 
 * @param qp_form 
 */
void PathPlanner::qp_gen(const TimeSeries& knots,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt,QP_form* qp_form){

    int n_seg = waypoints.poses.size() - 1;
    int poly_order = opt.poly_order;
    // this should be changed when we adopt single box 
    int n_var_total = 3 * (poly_order + 1) * n_seg;  // xyz
    int blck_size = poly_order+1;
    int blck_size_seg = 3*blck_size;  // stride along one segment of path 
    
    MatrixXd Q(n_var_total,n_var_total),H(1,n_var_total);
    Q.setZero(); H.setZero();

    MatrixXd Aeq(0,n_var_total),beq(0,1),Aineq(0,n_var_total),bineq(0,1);
    Aeq.setZero(); beq.setZero(); Aineq.setZero(); bineq.setZero();


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
            Q.block(blck_size_seg*(n),blck_size_seg*(n),blck_size_seg,blck_size_seg)=expand3(integral_jerk_squared(poly_order));
        }         
    }
    // if minimum snap 
    else if(opt.objective_derivative == 4){
        for (int n = 0; n < n_seg; n++) {
            MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
            double dn = knots[n + 1] - knots[n];
            // Q.block(blck_size*(n),blck_size*(n),blck_size,blck_size)=Dn*integral_snap_squared(poly_order)*Dn/pow(dn,7);
            Q.block(blck_size_seg*(n),blck_size_seg*(n),blck_size_seg,blck_size_seg)=expand3(integral_snap_squared(poly_order));
        }
    }else{
        cerr<<"undefined derivative in objective"<<endl;
    }

    
    /*
        2. Equality constraints  
    */
    //  -----------------------------------------------------------------------

    // (1) Initial constraints 


    // if it is soft, we include the deviation term 
    if (opt.is_waypoint_soft){    
            for(int n=0;n<n_seg;n++){
                MatrixXd Dn = time_scailing_mat(knots[n + 1]-knots[n], poly_order);
                int insert_start=blck_size_seg*(n);
                double time_scaling_factor;
                if (opt.objective_derivative == 3)
                    time_scaling_factor = pow(knots[n+1]-knots[n],5);
                else 
                    time_scaling_factor = pow(knots[n+1]-knots[n],7);

                //cout<<"time scaling factor: "<<time_scaling_factor<<endl;
                Q.block(insert_start,insert_start,blck_size_seg,blck_size_seg)+=expand3(MatrixXd(time_scaling_factor*opt.w_d*t_vec(poly_order,1,0)*t_vec(poly_order,1,0).transpose()));
                MatrixXd H_sub(1,blck_size_seg);
                H_sub << -2*time_scaling_factor*opt.w_d*(waypoints.poses[n+1].pose.position.x)*t_vec(poly_order,1,0).transpose(),
                         -2*time_scaling_factor*opt.w_d*(waypoints.poses[n+1].pose.position.y)*t_vec(poly_order,1,0).transpose(),
                         -2*time_scaling_factor*opt.w_d*(waypoints.poses[n+1].pose.position.z)*t_vec(poly_order,1,0).transpose();

                H.block(0,insert_start,1,blck_size_seg) = H_sub;             
                }
    }
       /*
        2. Equality constraints  
    */
    //  -----------------------------------------------------------------------

    // (1) Initial constraints 

    MatrixXd Aeq0(9,n_var_total),beq0_sub(9,1); Aeq0.setZero(); beq0_sub.setZero();
    Aeq0.block(0,0,9,blck_size_seg) = expand3(get_init_constraint_mat(waypoints.poses[0].pose.position.x,v0.linear.x,a0.linear.x,opt).A,
                   get_init_constraint_mat(waypoints.poses[0].pose.position.y,v0.linear.y,a0.linear.y,opt).A,
                   get_init_constraint_mat(waypoints.poses[0].pose.position.z,v0.linear.z,a0.linear.z,opt).A);
    
    row_append(Aeq,Aeq0); 

    beq0_sub  = row_stack3( get_init_constraint_mat(waypoints.poses[0].pose.position.x,v0.linear.x,a0.linear.x,opt).b,  
                                           get_init_constraint_mat(waypoints.poses[0].pose.position.y,v0.linear.x,a0.linear.y,opt).b,
                                           get_init_constraint_mat(waypoints.poses[0].pose.position.z,v0.linear.x,a0.linear.z,opt).b);

    row_append(beq,beq0_sub);
  

    // (2) Waypoints constraints (if it is hard constrained)

    if(not opt.is_waypoint_soft)
        for(int k = 0;k<n_seg;k++){            
            int insert_idx = k*blck_size_seg;
            MatrixXd Dn = time_scailing_mat(knots[k + 1]-knots[k], poly_order);
            MatrixXd Aeq_sub(3,n_var_total),beq_sub(3,1); Aeq_sub.setZero(); beq_sub.setZero();
            Aeq_sub.block(0,insert_idx,1,blck_size_seg) = expand3(t_vec(poly_order,1,0).transpose());
            row_append(Aeq,Aeq_sub); 

            beq_sub(0) = waypoints.poses[k+1].pose.position.x;
            beq_sub(1) = waypoints.poses[k+1].pose.position.y;
            beq_sub(2) = waypoints.poses[k+1].pose.position.z;
            row_append(beq,beq_sub);            
        }

 
    // (3) continuity constraints     
    // if the objective is snap, we will include 3rd order continuity (?) 
    for(int k = 0;k<n_seg-1;k++){
        int insert_idx = k*blck_size_seg;        
        MatrixXd Aeq_sub(3*3,n_var_total),beq_sub(3*3,1);
        Aeq_sub.setZero();
        Aeq_sub.block(0,insert_idx,3*3,2*blck_size_seg) = get_continuity_constraint_mat3(knots[k+1]-knots[k],knots[k+2]-knots[k+1],opt).A;
        beq_sub = get_continuity_constraint_mat3(knots[k+1]-knots[k],knots[k+2]-knots[k+1],opt).b;
        row_append(Aeq,Aeq_sub); row_append(beq,beq_sub);
    }

       

    /*
        3. Inequality constraints (only if there is a corridor constraint) 
    */
    //  -----------------------------------------------------------------------    
    int N_safe_pnts = opt.N_safe_pnts + 2;
    int n_ineq_consts = 3*2*(N_safe_pnts)*n_seg;

    MatrixXd A_sub(n_ineq_consts,n_var_total),b_sub(n_ineq_consts,1);
    A_sub.setZero(); b_sub.setZero(); 

    int ineq_row_insert_idx=0,ineq_col_insert_idx=0;    

    // flushing for new start
    if(safe_corridor_marker.points.size())
        safe_corridor_marker.points.clear();

    if(safe_corridor_marker_single_array.markers.size())
        safe_corridor_marker_single_array.markers.clear();


    safe_corridor_marker_single_base.header.frame_id = "world";
    safe_corridor_marker_single_base.ns = "sf_corridor";
    safe_corridor_marker_single_base.type = visualization_msgs::Marker::CUBE;
    safe_corridor_marker_single_base.action = 0;
    safe_corridor_marker_single_base.color.a = 0.5;
    safe_corridor_marker_single_base.color.r = 170.0/255.0;
    safe_corridor_marker_single_base.color.g = 1.0;
    safe_corridor_marker_single_base.color.b = 1.0;
    
   
    // per segment 
    for (int n = 0; n<n_seg ;n++){

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
        
        dx = (xf-x0)/((N_safe_pnts-1));
        dy = (yf-y0)/((N_safe_pnts-1));
        dz = (zf-z0)/((N_safe_pnts-1));

        Point p1,p2;
        p1.x = x0; p1.y = y0; p1.z = z0;
        p2.x = xf; p2.y = yf; p2.z = zf;
        Vector3d pnt1(p1.x,p1.y,p1.z),pnt2(p2.x,p2.y,p2.z); 
        Affine3d Twb = get_affine_corridor_pose(p1,p2);
        
        // pose     
        safe_corridor_marker_single_base.pose.position.x = Twb.translation()(0);                     
        safe_corridor_marker_single_base.pose.position.y = Twb.translation()(1);                     
        safe_corridor_marker_single_base.pose.position.z = Twb.translation()(2);                     

        Quaterniond q(Twb.rotation());
        safe_corridor_marker_single_base.pose.orientation.x = q.x();
        safe_corridor_marker_single_base.pose.orientation.y = q.y();
        safe_corridor_marker_single_base.pose.orientation.z = q.z();
        safe_corridor_marker_single_base.pose.orientation.w = q.w();
        
        float l = (pnt1 - pnt2).norm();
        // scale 
        double safe_r = opt.safe_r;
        safe_corridor_marker_single_base.scale.x = 2*(l/2 + safe_r);
        safe_corridor_marker_single_base.scale.y = 2*(safe_r);
        safe_corridor_marker_single_base.scale.z = 2*(safe_r);
        safe_corridor_marker_single_base.id = n;
        // append 
        safe_corridor_marker_single_array.markers.push_back(safe_corridor_marker_single_base);


        // per safe sample points along 
        for (int n_sub = 0 ; n_sub<N_safe_pnts;n_sub++){

            double x_sub,y_sub,z_sub;
            x_sub = x0 + dx*n_sub;
            y_sub = y0 + dy*n_sub;
            z_sub = z0 + dz*n_sub;
            double t_control = 1.0/(N_safe_pnts-1) * n_sub;
            
            // get A,b block for this control point 
            Constraint const_ineq = get_corridor_constraint_mat(p1,p2,t_vec(poly_order,t_control,0),opt);             

            A_sub.block(ineq_row_insert_idx,ineq_col_insert_idx,6,blck_size_seg) = const_ineq.A;
            b_sub.block(ineq_row_insert_idx,0,6,1) = const_ineq.b;

            // stride along row
            ineq_row_insert_idx += 6;        
        }

        // stride along col
        ineq_col_insert_idx += blck_size_seg;    
    }
    row_append(Aineq,A_sub);
    row_append(bineq,b_sub);

    QP_form qp; qp.Q = Q; qp.H = H; qp.A = Aineq; qp.b = bineq; qp.Aeq = Aeq; qp.beq = beq; 
    *qp_form = qp;
}

/**
 * @brief decoupled qp generation 
 * 
 * @param knots 
 * @param waypoints 
 * @param v0 
 * @param a0 
 * @param opt 
 * @param qp_form_xyz 
 */
void PathPlanner::qp_gen(const TimeSeries& knots,const nav_msgs::Path& waypoints,const geometry_msgs::Twist& v0,const geometry_msgs::Twist& a0,TrajGenOpts opt,QP_form_xyz* qp_form_xyz){



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
    // if the objective is snap, we will include 3rd order continuity (?)
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


    double safe_r = opt.safe_r;
    int N_safe_pnts = opt.N_safe_pnts;
    int n_ineq_consts = 2*(N_safe_pnts)*n_seg;

    /*
        3. Inequality constraints (only if there is a corridor constraint) 
    */
    //  -----------------------------------------------------------------------    

    if(opt.is_multi_corridor){
        // in case of multi corridor, the multiple cubes are used to represent the corridor region 
            cout << "[TRAJ_GEN]: multi corridor generator... " <<endl;
            // multi-corridor is called. if previous solve routine was single corridor, flush it 
            if(safe_corridor_marker_single_array.markers.size())
                safe_corridor_marker_single_array.markers.clear(); 

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
                
               double t_control = 1.0/(N_safe_pnts+1) * n_sub;

                
                Vector3d upper_limit,lower_limit;

                // axis parallel multiple cube
                    lower_limit(0) = (x_sub - safe_r);
                    lower_limit(1) = (y_sub - safe_r);
                    lower_limit(2) = (z_sub - safe_r);
                     
                    upper_limit(0) = (x_sub + safe_r);
                    upper_limit(1) = (y_sub + safe_r);
                    upper_limit(2) = (z_sub + safe_r);

                    

                    // marker update               
                    safe_corridor_marker.points[idx].x = x_sub;
                    safe_corridor_marker.points[idx].y = y_sub;
                    safe_corridor_marker.points[idx].z = z_sub;
                    idx++;

                // non parallel single rectangle
                

                A_sub.block(ineq_row_insert_idx,ineq_col_insert_idx1,1,blck_size)=-t_vec(poly_order,t_control,0).transpose();        
                            
                // lower limit 
                bx_sub.coeffRef(ineq_row_insert_idx) = -lower_limit(0);
                by_sub.coeffRef(ineq_row_insert_idx) = -lower_limit(1);
                bz_sub.coeffRef(ineq_row_insert_idx) = -lower_limit(2);        

                ineq_row_insert_idx++;

                // upper limit 
                A_sub.block(ineq_row_insert_idx,ineq_col_insert_idx1,1,blck_size)=t_vec(poly_order,t_control,0).transpose();        
                bx_sub.coeffRef(ineq_row_insert_idx) = upper_limit(0);
                by_sub.coeffRef(ineq_row_insert_idx) = upper_limit(1);
                bz_sub.coeffRef(ineq_row_insert_idx) = upper_limit(2);      

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
    
    *qp_form_xyz = qp_prob_xyz;
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
//	for(auto it = spline_xyz.knot_time.begin();it<spline_xyz.knot_time.end();it++)
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

geometry_msgs::Point point_eval_spline(PolySplineXYZ  spline_xyz,double t_eval){


    geometry_msgs::Point eval_point;

    int poly_order=spline_xyz.poly_order;
    // DEBUG
//			std::cout<<"knot time of this: "<<std::endl;
//    for(auto it = spline_xyz.knot_time.begin();it<spline_xyz.knot_time.end();it++)
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

geometry_msgs::Point vel_eval_spline(PolySplineXYZ  spline_xyz,double t_eval){


    geometry_msgs::Point eval_point;

    int poly_order=spline_xyz.poly_order;
    // DEBUG
//			std::cout<<"knot time of this: "<<std::endl;
//    for(auto it = spline_xyz.knot_time.begin();it<spline_xyz.knot_time.end();it++)
//		std::cout<<*it<<", ";spline_xyz
//	std::cout<<std::endl;
//    std::cout<<"point_eval: "<<t_eval.toSec()<<"knot time final: "<<spline.knot_time.back()<<std::endl;
    t_eval =min(spline_xyz.knot_time.back(),t_eval);
    t_eval =max(spline_xyz.knot_time.front(),t_eval);

    Eigen::Index spline_idx=find_spline_interval(spline_xyz.knot_time,t_eval);
//	std::cout<<"Index: "<<spline_idx<<std::endl;
    // double t_eval_norm = (t_eval-spline_xyz.knot_time[spline_idx])/(spline_xyz.knot_time[spline_idx+1]-spline_xyz.knot_time[spline_idx]);
    double t_eval_norm = (t_eval-spline_xyz.knot_time[spline_idx]);

    eval_point.x=t_vec(poly_order,t_eval_norm,1).transpose()*Map<VectorXd>(spline_xyz.spline_x.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_point.y=t_vec(poly_order,t_eval_norm,1).transpose()*Map<VectorXd>(spline_xyz.spline_y.poly_coeff[spline_idx].coeff.data(),poly_order+1);
    eval_point.z=t_vec(poly_order,t_eval_norm,1).transpose()*Map<VectorXd>(spline_xyz.spline_z.poly_coeff[spline_idx].coeff.data(),poly_order+1);

    return eval_point;
}


nav_msgs::Path horizon_eval_spline(PolySplineXYZ spline_xyz,int N_eval_interval){


    geometry_msgs::PoseStamped poseStamped;
    nav_msgs::Path path;

    // ROS_INFO("evaluating spline....");

    int n_seg=spline_xyz.n_seg;
    int poly_order = spline_xyz.poly_order;

    // printf("n_seg : %d, poly_order: %d, \n",spline_xyz.n_seg,spline_xyz.poly_order);

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


    return path;
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
    //std::cout<<"hessian type: "<<qp_obj.getHessianType()<<endl;
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

    //cout << "solution" <<endl;
    //cout << sol <<endl;
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

PolySplineXYZ PathPlanner::get_solution_couple(VectorXd sol,int poly_order, int n_seg){

    int D = 3; // xyz
    vector<PolySpline> polySplineXYZ(D);
    int blck_size = poly_order + 1;
    int blck_size_seg = blck_size *3;

    for(int d = 0; d<D ; d++){
        polySplineXYZ[d].n_seg = n_seg; polySplineXYZ[d].poly_coeff.reserve(n_seg);

        for(uint i = 0; i<polySplineXYZ[d].n_seg;i++)
            polySplineXYZ[d].poly_coeff[i].poly_order=poly_order;
        int n_var=n_seg*(poly_order+1);

        // std::cout<<"[DEBUG] solution:"<<std::endl;
        // std::cout<<var<<std::endl;
        // from lowest order 0
        for(int n=0;n<n_seg;n++){
            PolyCoeff coeff;
            coeff.coeff.resize(poly_order+1);
            for(int i=0;i<blck_size;i++){
                coeff.coeff[i]=sol(n*blck_size_seg+d*blck_size+i); 
                coeff.poly_order = poly_order;         
            }
            polySplineXYZ[d].poly_coeff.push_back(coeff);
        }
    }

    PolySplineXYZ spline_xyz_temp;
    spline_xyz_temp.spline_x = polySplineXYZ[0];
    spline_xyz_temp.spline_y = polySplineXYZ[1];
    spline_xyz_temp.spline_z = polySplineXYZ[2];

    return spline_xyz_temp;
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
    
    int N_constraint = opt.objective_derivative;
    int poly_order = opt.poly_order;
    MatrixXd Aeq(N_constraint,2*(poly_order+1));
    MatrixXd beq(N_constraint,1); beq.setZero();   
    Aeq.setZero();
    beq.setZero();
    MatrixXd D1 = time_scailing_mat(dt1,poly_order);
    MatrixXd D2 = time_scailing_mat(dt2,poly_order);

    // 0th order         
    // Aeq. block(0,0,1,poly_order+1) = t_vec(poly_order,1,0).transpose()*D1;
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

     if (N_constraint == 4){    
        // 3rd order
        Aeq.block(3,0,1,poly_order+1) = t_vec(poly_order,1,3).transpose()*pow(dt2,3);
        Aeq.block(3,poly_order+1,1,poly_order+1) = -t_vec(poly_order,0,3).transpose()*pow(dt1,3); 
    }

    Constraint constraint;
    constraint.A = Aeq;
    constraint.b = beq;
    
    return constraint;
}

// output size 3x(2(N+1))
Constraint PathPlanner::get_continuity_constraint_mat3(double dt1,double dt2,TrajGenOpts opt){
    
    int N_constraint = 3*3;
    int poly_order = opt.poly_order;
    int blck_size = poly_order + 1;
    int blck_size_seg = blck_size * 3;
    MatrixXd Aeq(N_constraint,3*2*(poly_order+1));
    MatrixXd beq(N_constraint,1);   
    Aeq.setZero();
    beq.setZero();
    MatrixXd D1 = time_scailing_mat(dt1,poly_order);
    MatrixXd D2 = time_scailing_mat(dt2,poly_order);


    int insert_row,insert_col1,insert_col2; 
    
    for (int i = 0; i<3 ;i++){
        
        insert_row = 3*i; insert_col1 = blck_size*i,insert_col2 = blck_size_seg + insert_col1;
        
        // 0th order
        Aeq.block(insert_row,insert_col1,1,blck_size) = t_vec(poly_order,1,0).transpose();
        Aeq.block(insert_row,insert_col2,1,blck_size) = -t_vec(poly_order,0,0).transpose();
        
        // 1th order
        Aeq.block(insert_row+1,insert_col1,1,blck_size) = t_vec(poly_order,1,1).transpose()*dt2;
        Aeq.block(insert_row+1,insert_col2,1,blck_size) = -t_vec(poly_order,0,1).transpose()*dt1;
        
         // 2nd order
        Aeq.block(insert_row+2,insert_col1,1,blck_size) = t_vec(poly_order,1,2).transpose()*pow(dt2,2);
        Aeq.block(insert_row+2,insert_col2,1,blck_size) = -t_vec(poly_order,0,2).transpose()*pow(dt1,2);
         
    }

    Constraint constraint;
    constraint.A = Aeq;
    constraint.b = beq;
    
    return constraint;
}


/**
 * @brief 
 * 
 * @param pnt1 
 * @param pnt2 
 * @param t_vec 
 * @param option 
 * @return Constraint 
 */
Constraint PathPlanner::get_corridor_constraint_mat(Point pnt1 ,Point pnt2,VectorXd t_vec,TrajGenOpts option){

    Affine3d Twb = get_affine_corridor_pose(pnt1,pnt2);
    int blck_size = t_vec.size(); // 3* blck_size  = blck_size_seg

    Vector3f p1(pnt1.x,pnt1.y,pnt1.z),p2(pnt2.x,pnt2.y,pnt2.z);  
    float l = (p1 - p2).norm();

    MatrixXd A_sub(3,3*blck_size),b(3,1);
    Matrix3d Rbw = Twb.rotation().transpose();
    Vector3d twb = Twb.translation();



    // matrix A_sub
    for (int r = 0 ; r<3 ; r++)
        for(int c = 0 ; c<3 ; c++)
            A_sub.block(r,c*blck_size,1,blck_size) = Rbw(r,c) * t_vec.transpose();


    Vector3d upper_limit;
    upper_limit << l/2 + option.safe_r , option.safe_r , option.safe_r;
    Vector3d lower_limit = -upper_limit;

    upper_limit += Rbw * twb;
    lower_limit += Rbw * twb;

    Constraint constraint;
    constraint.A = MatrixXd(6,3*blck_size); constraint.b = MatrixXd(6,1);
    constraint.A << A_sub,-A_sub ;
    constraint.b << upper_limit, -lower_limit;
    
    return constraint;    
}

/**
 * @brief write current PolysplineXYZ
 * @param file_name
 */
void PathPlanner::write_spline(string file_name) {

    // write meta data

    int n_seg = spline_xyz.n_seg;
    int poly_order = spline_xyz.poly_order;

    ofstream fStream;
    fStream.open(file_name);
    fStream << n_seg <<" " << poly_order << endl;
    for (auto knot : spline_xyz.knot_time)
        fStream << knot <<" ";
    fStream<<endl;

    // polynomial coefficient

    for(int k=0;k<n_seg;k++) {
        for (int n = 0; n <= poly_order; n++) {
            fStream << spline_xyz.spline_x.poly_coeff[k].coeff[n] << " ";
        }
        fStream << endl;
        for (int n = 0; n <= poly_order; n++) {
            fStream << spline_xyz.spline_y.poly_coeff[k].coeff[n]<<" ";
        }
        fStream << endl;
        for (int n = 0; n <= poly_order; n++) {
            fStream << spline_xyz.spline_z.poly_coeff[k].coeff[n]<<" ";
        }
        fStream << endl;
    }
}

MatrixXd expand3(MatrixXd small_mat){
    const int r_small = small_mat.rows() ,c_small =small_mat.cols();
    MatrixXd new_mat(3*r_small,3*c_small);
    new_mat.setZero();
    new_mat.block(0,0,r_small,c_small) = small_mat;
    new_mat.block(r_small,c_small,r_small,c_small) = small_mat;
    new_mat.block(2*r_small,2*c_small,r_small,c_small) = small_mat;
    return new_mat;
}

MatrixXd expand3(MatrixXd small_mat1,MatrixXd small_mat2,MatrixXd small_mat3){
    const int r_small = small_mat1.rows() ,c_small =small_mat1.cols();
    MatrixXd new_mat(3*r_small,3*c_small); new_mat.setZero();
    new_mat.block(0,0,r_small,c_small) = small_mat1;
    new_mat.block(r_small,c_small,r_small,c_small) = small_mat2;
    new_mat.block(2*r_small,2*c_small,r_small,c_small) = small_mat3;
    return new_mat;
}

MatrixXd row_stack3(MatrixXd mat1){
    MatrixXd new_mat(3*mat1.rows(),mat1.cols());
    new_mat.block(0,0,mat1.rows(),mat1.cols()) = mat1;
    new_mat.block(mat1.rows(),0,mat1.rows(),mat1.cols()) = mat1;
    new_mat.block(2*mat1.rows(),0,mat1.rows(),mat1.cols()) = mat1;
    return new_mat;
}


MatrixXd row_stack3(MatrixXd mat1,MatrixXd mat2,MatrixXd mat3){
    MatrixXd new_mat(3*mat1.rows(),mat1.cols());

    new_mat.block(0,0,mat1.rows(),mat1.cols()) = mat1;
    new_mat.block(mat1.rows(),0,mat1.rows(),mat1.cols()) = mat2;
    new_mat.block(2*mat1.rows(),0,mat1.rows(),mat1.cols()) = mat3;
    return new_mat;
}

/**
MatrixXd expand3(VectorXd small_vec){
    int len_small = small_vec.size();
    MatrixXd new_mat(1,3*len_small); new_mat.setZero();
    new_mat.block<0,0,1,len_small> = small_vec;
    new_mat.block<0,len_small,1,len_small> = small_vec;
    new_mat.block<0,2*len_small,1,len_small> = small_vec;
    return new_mat;
}
**/
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



